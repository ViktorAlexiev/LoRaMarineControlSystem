#include <Arduino.h>
#include <LoRa.h>
#include "PinChangeInterrupt.h"


#define BOATID 0x8787
#define FBBUFFSIZE 2
#define MAXRETRIES 5
#define hod_svetl 3
#define sirenaa 4
#define zadna_p A0
#define predna_p A1
#define rudann 6
#define rudan_allow 5
#define led_pin 7

enum modules{
  preden=1,
  zaden
};

enum konsumatori{
  hodovi=1,
  sirena,
  zadnaP,
  prednaP,
  rudan
};

enum commands{
  OFF = 0,
  ON = 1
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
  uint8_t konsumator;
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
    p->konsumator  = 0;
    p->command  = 0;
}

void handleIdle(){
}

void handleError(const char* msg){
  Serial.println(msg);
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

void sendCommand(uint8_t moduleID, uint8_t konsumator, uint8_t command){
  myPacket pack = {BOATID, moduleID, konsumator, command};
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
    currentState = AWAITFEEDBACK;
    
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

void isr_hodovi() {
  if (digitalRead(hod_svetl) == LOW)
    sendCommand(zaden, hodovi, ON);
  else
    sendCommand(zaden, hodovi, OFF);
}

void isr_sirena() {
  if (digitalRead(sirena) == LOW)
    sendCommand(zaden, sirena, ON);
  else
    sendCommand(zaden, sirena, OFF);
}

void isr_zadnaP() {
  if (digitalRead(zadna_p) == LOW)
    sendCommand(zaden, zadnaP, ON);
  else
    sendCommand(zaden, zadnaP, OFF);
}

void isr_prednaP() {
  if (digitalRead(predna_p) == LOW)
    sendCommand(preden, prednaP, ON);
  else
    sendCommand(preden, prednaP, OFF);
}

bool allow_rudan = 0;

void isr_rudan() {
  if (digitalRead(rudan) == LOW) {
    if (allow_rudan) {
      sendCommand(preden, rudan, ON);
    }
  } else {
    sendCommand(preden, rudan, OFF);
  }
}

void isr_rudan_allow() {
  if (digitalRead(rudan) == LOW) {
    allow_rudan = 1;
  } else {
    allow_rudan = 0;
    sendCommand(preden, rudan, OFF);
  }
}

void setup(){
  Serial.begin(115200);

  // set all pins as INPUT_PULLUP
  pinMode(hod_svetl,   INPUT_PULLUP);
  pinMode(sirenaa,      INPUT_PULLUP);
  pinMode(zadna_p,     INPUT_PULLUP);
  pinMode(predna_p,    INPUT_PULLUP);
  pinMode(rudann,       INPUT_PULLUP);
  pinMode(rudan_allow, INPUT_PULLUP);
  pinMode(led_pin,     OUTPUT);

  // attach pin change interrupts
  attachPCINT(digitalPinToPCINT(hod_svetl),  isr_hodovi,  CHANGE);
  attachPCINT(digitalPinToPCINT(sirena),     isr_sirena,  CHANGE);
  attachPCINT(digitalPinToPCINT(zadna_p),    isr_zadnaP,  CHANGE);
  attachPCINT(digitalPinToPCINT(predna_p),   isr_prednaP, CHANGE);
  attachPCINT(digitalPinToPCINT(rudan),      isr_rudan,   CHANGE);
  attachPCINT(digitalPinToPCINT(rudan_allow),      isr_rudan_allow,   CHANGE);

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
