#include <LoRa.h>

#define BOATID 0x8787

enum commands{
  chervena,
  zelena,
  bqla,
  sirena,
  zadna,
  predna,
  rudan
};

enum states{
  MYIDLE,
  AWAITFEEDBACK
};

enum states currentState = MYIDLE;

struct myPacket {
  uint32_t boatID;
  uint8_t moduleID;
  uint8_t command;
};

void handleIdle(){
}

void sendcommand(uint8_t moduleID, uint8_t command){
  myPacket pack = {BOATID, moduleID, command };
  
  LoRa.beginPacket();
  LoRa.print(pack);
  LoRa.endPacket();
}

void awaitFeedback(){
}

void handleError(char *msg){
  Serial.println(msg);
}

void setup(){
  Serial.begin(115200);

  if(!LoRa.begin(433E6)){
      Serial.println("LoRa transmitter not working");
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
}

void loop(){
  switch(currentState) {
    case 0:
      handleIdle();
      break;
    case 1:
      awaitFeedback();
      break;
      
  }
}
