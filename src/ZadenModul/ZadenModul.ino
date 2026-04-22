#include <Arduino.h>
#include <LoRa.h>

#define BOATID  0x87878787
#define MODUL   2

#define hod_svetl A0
#define sirenaa   4
#define zadna_p   3

int pins[3] = {hod_svetl, sirenaa, zadna_p};

enum konsumatori { hodovi = 1, sirena, zadnaP };

struct __attribute__((packed)) myPacket {
  uint32_t boatID;
  uint8_t  moduleID;
  uint8_t  konsumator;
  uint8_t  command;
};

// ─────────────────────────────────────────────
//  RX БУФЕР  –  получените пакети се складират тук
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
//  ACK ОПАШКА  –  ACK-овете чакат 500ms преди да излетят
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
//  RADIO RECEIVE POLL  –  чете от radio → RX буфер
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
//  PROCESS RX  –  обработва пакетите от RX буфера
// ─────────────────────────────────────────────
void processRx() {
  myPacket p;
  while (rxPop(&p)) {
    // Приложи командата
    digitalWrite(pins[p.konsumator - 1], p.command);
    //Serial.print("[CMD] kons=%u cmd=%u\n", p.konsumator, p.command);

    // Добави ACK в опашката (праща след 500ms)
    ackEnqueue(p, millis() + 500);
  }
}

// ─────────────────────────────────────────────
//  ACK MANAGER  –  праща ACK-овете когато им дойде времето
// ─────────────────────────────────────────────
void ackManager() {
  unsigned long now = millis();
  for (uint8_t i = 0; i < ackCount; ) {
    if (now >= ackSendAt[i]) {
      LoRa.beginPacket();
      LoRa.write((uint8_t*)&ackQueue[i], sizeof(ackQueue[i]));
      LoRa.endPacket();
      LoRa.receive();
      //Serial.print("[ACK] kons=%u cmd= sent\n", ackQueue[i].konsumator, ackQueue[i].command);

      // Изтрий от масива (shift)
      for (uint8_t j = i; j < ackCount - 1; j++) {
        ackQueue[j]  = ackQueue[j + 1];
        ackSendAt[j] = ackSendAt[j + 1];
      }
      ackCount--;
      // не увеличаваме i – трябва да проверим новия елемент на позиция i
    } else {
      i++;
    }
  }
}

// ─────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial);

  for (int i = 0; i < 3; i++) pinMode(pins[i], OUTPUT);

  while (!LoRa.begin(433E6)) Serial.print("LoRa could not start");
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();
}

void loop() {
  radioReceivePoll();  // 1. чети от radio → RX буфер
  processRx();         // 2. обработвай RX буфера
  ackManager();        // 3. праща ACK-ове навреме
}
