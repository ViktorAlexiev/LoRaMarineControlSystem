#include <Arduino.h>
#include <LoRa.h>

#define BOATID 0x8787
#define MODUL 2

#define hod_svetl A0
#define sirenaa 4
#define zadna_p 3

int pins[3] = {hod_svetl, sirenaa, zadna_p};

enum konsumatori{
  hodovi =1,
  sirena,
  zadnaP
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
  myPacket myData;
  int len = LoRa.readBytes((uint8_t*)&myData, sizeof(myData));

  if (len != sizeof(myData)) {
    // incomplete packet → discard
    return;
  }
  LoRa.readBytes((uint8_t*)&myData, sizeof(myData));

  if(myData.boatID != BOATID || myData.moduleID != MODUL){ return;}

  digitalWrite(pins[myData.konsumator-1], myData.command);
}
