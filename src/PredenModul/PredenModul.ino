#include <Arduino.h>
#include <LoRa.h>

#define BOATID      0x87878787
#define MODUL       1

#define predna_P    3
#define rudan_R     A0
#define rudan_B     4
#define pins_count  3

int pins[3] = {predna_P, rudan_R, rudan_B};

enum konsumatori { prednaP = 4, rudan = 5 };

struct myPacket {
  uint32_t boatID;
  uint8_t  moduleID;
  uint8_t  konsumator;
  uint8_t  command;
};

// ─────────────────────────────────────────────
//  RX БУФЕР
// ─────────────────────────────────────────────
#define RXBUFFSIZE 8

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

bool rxPop(struct myPacket *out) {
  if (rxCount == 0) return false;
  *out = rxBuffer[rxHead];
  rxHead = (rxHead + 1) % RXBUFFSIZE;
  rxCount--;
  return true;
}

// ─────────────────────────────────────────────
//  ACK ОПАШКА
// ─────────────────────────────────────────────
#define ACKBUFFSIZE 8

struct myPacket ackQueue[ACKBUFFSIZE];
unsigned long   ackSendAt[ACKBUFFSIZE] = {0};
uint8_t         ackCount = 0;

bool ackEnqueue(struct myPacket p, unsigned long sendAt) {
  if (ackCount >= ACKBUFFSIZE) { Serial.println("[ERR] ACK queue full"); return false; }
  ackQueue[ackCount]  = p;
  ackSendAt[ackCount] = sendAt;
  ackCount++;
  return true;
}

// ─────────────────────────────────────────────
//  RADIO RECEIVE POLL
// ─────────────────────────────────────────────
void radioReceivePoll() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(myPacket)) {
    myPacket p;
    LoRa.readBytes((uint8_t*)&p, sizeof(p));
    if (p.boatID == BOATID && p.moduleID == MODUL) {
      rxPush(p);
    }
  } else if (packetSize > 0) {
    while (LoRa.available()) LoRa.read();
  }
}

// ─────────────────────────────────────────────
//  PROCESS RX
// ─────────────────────────────────────────────
void processRx() {
  myPacket p;
  while (rxPop(&p)) {

    if (p.konsumator != rudan) {
      int idx = p.konsumator - 4;  // prednaP=4 → idx=0
      digitalWrite(pins[idx], p.command);
      //Serial.printf("[CMD] pin idx=%d cmd=%u\n", idx, p.command);
    } else {
      // rudan управлява два пина
      for (int i = pins_count - 2; i < pins_count; i++) {
        digitalWrite(pins[i], p.command);
      }
      //Serial.printf("[CMD] rudan cmd=%u\n", p.command);
    }

    // Насрочи ACK след 500ms
    ackEnqueue(p, millis() + 500);
  }
}

// ─────────────────────────────────────────────
//  ACK MANAGER
// ─────────────────────────────────────────────
void ackManager() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < ackCount; ) {
    if (now >= ackSendAt[i]) {
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&ackQueue[i], sizeof(ackQueue[i]));
      LoRa.endPacket();
      LoRa.receive();
      //Serial.printf("[ACK] kons=%u cmd=%u sent\n", ackQueue[i].konsumator, ackQueue[i].command);

      for (uint8_t j = i; j < ackCount - 1; j++) {
        ackQueue[j]  = ackQueue[j + 1];
        ackSendAt[j] = ackSendAt[j + 1];
      }
      ackCount--;
    } else {
      i++;
    }
  }
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < pins_count; i++) pinMode(pins[i], OUTPUT);

  while (!LoRa.begin(433E6)) Serial.print("LoRa could not start");
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  radioReceivePoll();
  processRx();
  ackManager();
}
