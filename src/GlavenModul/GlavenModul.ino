#include <Arduino.h>
#include <LoRa.h>

#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   14
#define LORA_DIO0  26

#define BOATID         0x87878787
#define FBBUFFSIZE     8      // feedback (TX) buffer size
#define RXBUFFSIZE     8      // receive (RX) buffer size
#define MAXRETRIES     10
#define TX_DELAY_MS    500    // чака 500ms преди да праща следващото от опашката

enum modules   { preden = 1, zaden };
enum moduleStatus { working = 1, communication_error };

struct ModuleStatus {
  uint8_t preden = working;
  uint8_t zaden  = working;
};

struct ModuleStatus modStatus;

enum konsumatori { hodovi = 1, sirena, zadnaP, prednaP, rudan };
enum commands    { OFF = 0, ON = 1 };

struct __attribute__((packed)) myPacket {
  uint32_t boatID;
  uint8_t  moduleID;
  uint8_t  konsumator;
  uint8_t  command;
};

const struct myPacket ZERO_PACKET = {0};

// ─────────────────────────────────────────────
//  TX ОПАШКА  (sendCommand добавя тук, manager праща)
// ─────────────────────────────────────────────
struct myPacket  txQueue[FBBUFFSIZE];
uint8_t          txHead = 0;   // следващото за пращане
uint8_t          txTail = 0;   // следващото свободно място
uint8_t          txCount = 0;

bool txQueuePush(struct myPacket p) {
  if (txCount >= FBBUFFSIZE) return false;
  txQueue[txTail] = p;
  txTail = (txTail + 1) % FBBUFFSIZE;
  txCount++;
  return true;
}

bool txQueuePop(struct myPacket *out) {
  if (txCount == 0) return false;
  *out = txQueue[txHead];
  txHead = (txHead + 1) % FBBUFFSIZE;
  txCount--;
  return true;
}

// ─────────────────────────────────────────────
//  FEEDBACK БУФЕР  (очаква ACK за изпратен пакет)
// ─────────────────────────────────────────────
struct myPacket  feedbackBuffer[FBBUFFSIZE];
unsigned long    fbTimestamp[FBBUFFSIZE]  = {0};
uint8_t          fbRetries[FBBUFFSIZE]    = {0};
const long       retryInterval = 2000;

int fbFindEmpty() {
  for (int i = 0; i < FBBUFFSIZE; i++)
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(myPacket)) == 0)
      return i;
  return -1;
}

void fbAdd(struct myPacket p) {
  int idx = fbFindEmpty();
  if (idx == -1) { Serial.println("[ERR] feedback buffer full"); return; }
  feedbackBuffer[idx] = p;
  fbTimestamp[idx]    = millis();
  fbRetries[idx]      = 0;
}

void fbClear(int idx) {
  memset(&feedbackBuffer[idx], 0, sizeof(myPacket));
  fbTimestamp[idx] = 0;
  fbRetries[idx]   = 0;
}

// ─────────────────────────────────────────────
//  RX БУФЕР  (получени пакети чакат да бъдат обработени)
// ─────────────────────────────────────────────
struct myPacket rxBuffer[RXBUFFSIZE];
uint8_t rxHead  = 0;
uint8_t rxTail  = 0;
uint8_t rxCount = 0;

bool rxPush(struct myPacket p) {
  if (rxCount >= RXBUFFSIZE) { Serial.println("[ERR] RX buffer full"); return false; }
  rxBuffer[rxTail] = p;
  rxTail = (rxTail + 1) % RXBUFFSIZE;
  rxCount++;
  return true;
}

// Търси в RX буфера пакет който съвпада с очаквания ACK.
// Ако го намери – го маха и връща true.
bool rxFind(struct myPacket expected) {
  for (uint8_t i = 0; i < rxCount; i++) {
    uint8_t realIdx = (rxHead + i) % RXBUFFSIZE;
    if (memcmp(&rxBuffer[realIdx], &expected, sizeof(myPacket)) == 0) {
      // Изтриваме намерения елемент (shift)
      for (uint8_t j = i; j < rxCount - 1; j++) {
        uint8_t cur  = (rxHead + j)     % RXBUFFSIZE;
        uint8_t next = (rxHead + j + 1) % RXBUFFSIZE;
        rxBuffer[cur] = rxBuffer[next];
      }
      rxCount--;
      rxTail = (rxTail == 0) ? RXBUFFSIZE - 1 : rxTail - 1;
      return true;
    }
  }
  return false;
}

// ─────────────────────────────────────────────
//  RADIO RECEIVE  –  чете всичко налично и пълни RX буфера
// ─────────────────────────────────────────────
void radioReceivePoll() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(myPacket)) {
    myPacket p;
    LoRa.readBytes((uint8_t*)&p, sizeof(p));
    Serial.printf("[RX] boatID=%08X mod=%u kons=%u cmd=%u\n",
                  p.boatID, p.moduleID, p.konsumator, p.command);
    rxPush(p);
  } else if (packetSize > 0) {
    while (LoRa.available()) LoRa.read(); // изхвърли боклук
  }
}

// ─────────────────────────────────────────────
//  TX MANAGER  –  вика се от loop(), праща по един пакет наведнъж
// ─────────────────────────────────────────────
unsigned long lastTxTime = 0;
bool          waitingForAck = false;   // не прати следващото докато не е ACK-нат текущото

void txManager() {
  if (waitingForAck) return;           // изчакай ACK преди нов пакет
  if (txCount == 0) return;            // нищо в опашката

  unsigned long now = millis();
  if (now - lastTxTime < TX_DELAY_MS) return;  // изчакай 500ms между пращанията

  myPacket p;
  if (!txQueuePop(&p)) return;

  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.receive();

  fbAdd(p);
  lastTxTime  = now;
  waitingForAck = true;

  Serial.printf("[TX] mod=%u kons=%u cmd=%u\n", p.moduleID, p.konsumator, p.command);
}

// ─────────────────────────────────────────────
//  FEEDBACK MANAGER  –  проверява RX буфера за ACK-ове, retry при нужда
// ─────────────────────────────────────────────
void feedbackManager() {
  unsigned long now = millis();
  bool anyPending   = false;

  for (int i = 0; i < FBBUFFSIZE; i++) {
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(myPacket)) == 0) continue;
    anyPending = true;

    // Провери дали вече сме получили ACK в RX буфера
    if (rxFind(feedbackBuffer[i])) {
      Serial.printf("[ACK] mod=%u kons=%u cmd=%u\n",
                    feedbackBuffer[i].moduleID,
                    feedbackBuffer[i].konsumator,
                    feedbackBuffer[i].command);
      if (feedbackBuffer[i].moduleID == preden)
        modStatus.preden = communication_error;
      else
        modStatus.zaden = communication_error;
      fbClear(i);
      waitingForAck = false;  // може да праща следващото
      continue;
    }

    // Timeout / retry
    if (now - fbTimestamp[i] >= retryInterval) {
      if (fbRetries[i] >= MAXRETRIES) {
        Serial.printf("[ERR] No ACK mod=%u kons=%u\n",
                      feedbackBuffer[i].moduleID, feedbackBuffer[i].konsumator);
        if (feedbackBuffer[i].moduleID == preden)
          modStatus.preden = communication_error;
        else
          modStatus.zaden = communication_error;
        fbClear(i);
        waitingForAck = false;
      } else {
        // Retry с random jitter
        fbTimestamp[i] = now + random(0, 150);
        fbRetries[i]++;
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&feedbackBuffer[i], sizeof(feedbackBuffer[i]));
        LoRa.endPacket();
        LoRa.receive();
        Serial.printf("[RETRY %u] mod=%u kons=%u\n",
                      fbRetries[i],
                      feedbackBuffer[i].moduleID,
                      feedbackBuffer[i].konsumator);
      }
    }
  }

  if (!anyPending) waitingForAck = false;
}

// ─────────────────────────────────────────────
//  PUBLIC API  –  просто добавя в опашката, НЕ праща веднага
// ─────────────────────────────────────────────
void sendCommand(uint8_t moduleID, uint8_t konsumator, uint8_t command) {
  myPacket p = {BOATID, moduleID, konsumator, command};
  if (!txQueuePush(p)) {
    Serial.println("[ERR] TX queue full");
  }
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) Serial.println("LoRa transmitter not working");
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  // Всички команди се добавят в опашката – txManager ги праща по ред с 500ms пауза
  sendCommand(zaden, hodovi,  ON);
  sendCommand(zaden, sirena,  OFF);
  sendCommand(zaden, zadnaP,  ON);
  sendCommand(preden, prednaP, ON);
  sendCommand(preden, rudan,   ON);
}

void loop() {
  radioReceivePoll();   // 1. чети каквото е дошло → RX буфер
  feedbackManager();    // 2. обработвай ACK-ове / retry
  txManager();          // 3. праща следващото от TX опашката ако може
}
