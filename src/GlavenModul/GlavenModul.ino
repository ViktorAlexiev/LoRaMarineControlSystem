#include <LoRa.h>

#define BOATID 0x8787
#define FBBUFFSIZE 2
#define MAXRETRIES 5;

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

/*struct awaitedPacket {
enum errorCodes{
  ELECTRICAL_FAILURE
};      да се имплементира система за приемане на информация за грешки */


struct myPacket {
  uint32_t boatID;
  uint8_t moduleID;
  uint8_t command;
};


const struct myPacket ZERO_PACKET = {0};

void resetPacket(struct myPacket *p)
{
    if (p == NULL) {
        return; // safety check
    }

    p->boatID   = 0;
    p->moduleID = 0;
    p->command  = 0;
}

void handleIdle(){
}

int findFirstZeroed(struct myPacket arr[], size_t len)
{
    struct myPacket zero = {0};

    for (size_t i = 0; i < len; i++) {
        if (memcmp(&arr[i], &zero, sizeof(struct myPacket)) == 0) {
            return (int)i;  // found first zeroed element
        }
    }

    return -1; // none found
}

myPacket feedbackBuffer[FBBUFFSIZE];
unsigned long previousMillis[FBBUFFSIZE] = {0};  
const long interval = 1000;        

uint8_t bufferRetries[FBBUFFSIZE] = {0};  

void sendcommand(uint8_t moduleID, uint8_t command){
  myPacket pack = {BOATID, moduleID, command };
  
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&pack, sizeof(pack));
  LoRa.endPacket();
  int index = findFirstZeroed(feedbackBuffer, sizeof(feedbackBuffer)/sizeof(feedbackBuffer[0]));
  if(index != -1){
    feedbackBuffer[index] = pack;
    bufferRetries[index] = 0;
    int maxValue = previousMillis[0];
    for (int i = 1; i < FBBUFFSIZE; i++) {
        if (previousMillis[i] > maxValue) {
            maxValue = previousMillis[i];
        }
    }
    
    if(maxValue!=0){
      previousMillis[index] = maxValue+100;
    }else{
      previousMillis[index] = millis();
    }
    
  }else{
    handleError("feedback buffer is full");
  }
  
}

void awaitFeedback(myPacket *feedbackBuffer){
  bool returnToIdle = 1;
  bool feedbackRecieved[FBBUFFSIZE] = {0, 0};
  for(int i=0; i<FBBUFFSIZE; i++){
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(struct myPacket)) != 0) {
        returnToIdle = 0;
    }else{
        feedbackRecieved[i] = 1;
        
    }
  }

  if(returnToIdle){
    currentState = MYIDLE;
  }
  
  myPacket myData;
  LoRa.readBytes((uint8_t*)&myData, sizeof(myData));
  unsigned long currentMillis = millis();

  for(int i=0; i<FBBUFFSIZE; i++){
    if(memcmp(&feedbackBuffer[i], &myData, sizeof(struct myPacket)) != 0){
       resetPacket(&feedbackBuffer[i]);
       feedbackRecieved[i] = 1;
       previousMillis[i] = 0;
       bufferRetries[i] = 0;
    }

    if(bufferRetries[i] >= 5){ // да се смени с макрото ама не знам защо дава expected primary-expression before ')' token
       handleError("Error in radio communication");
       resetPacket(&feedbackBuffer[i]);
       feedbackRecieved[i] = 1;
       previousMillis[i] = 0;
       bufferRetries[i] = 0;
    }

    if(feedbackRecieved[i] != 1 && bufferRetries[i]<5){
      if (currentMillis - previousMillis[i] >= interval) {
        previousMillis[i] = currentMillis;
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&feedbackBuffer[i], sizeof(feedbackBuffer[i]));
        LoRa.endPacket();
        bufferRetries[i]+=1;
      }
    }
  }
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
      awaitFeedback(feedbackBuffer);
      break;
    default:
      break;
  }
}
