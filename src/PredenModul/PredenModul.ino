#include <Arduino.h>
#include <LoRa.h>

<<<<<<< HEAD
#define BOATID 0x8787
=======
#define BOATID 0x87878787
>>>>>>> a2e405b (fixed bugs)
#define MODUL 1

#define predna_P 3
#define rudan_R A0
#define rudan_B 4
#define pins_count 3

int pins[3] = {predna_P, rudan_R, rudan_B};

enum konsumatori{
  prednaP = 4,
  rudan = 5
};

struct myPacket {
  uint32_t boatID;
  uint8_t moduleID;
  uint8_t konsumator;
  uint8_t command;
};

void setup() {
  Serial.begin(115200);
  while(!Serial);

  for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
    pinMode(pins[i], OUTPUT);
  }

  while(!LoRa.begin(433E6)){
    Serial.print("LoRa could not start");
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
}

void loop() {
    handle_recieve();
}

void handleError(const char* msg){
  Serial.println(msg);
}

void handle_recieve(){
<<<<<<< HEAD
  myPacket myData;
  int len = LoRa.readBytes((uint8_t*)&myData, sizeof(myData));

  if (len != sizeof(myData)) {
    // incomplete packet → discard
=======
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  
  myPacket myData;
  
  if (packetSize != sizeof(myData)) {
    while (LoRa.available()) LoRa.read();
    return;
  }

  if (packetSize != sizeof(myData)) {
>>>>>>> a2e405b (fixed bugs)
    return;
  }
  LoRa.readBytes((uint8_t*)&myData, sizeof(myData));

  if(myData.boatID != BOATID || myData.moduleID != MODUL){ return;}
<<<<<<< HEAD
=======
  
>>>>>>> a2e405b (fixed bugs)
  if(myData.konsumator != rudan){
    digitalWrite(pins[myData.konsumator-4], myData.command);
  }else{
    for(int i = pins_count-1; i>pins_count-3; i++){
      digitalWrite(pins[1], myData.command);
      digitalWrite(pins[2], myData.command);
    }
  }
}
