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

/*
enum errorCodes{
  ELECTRICAL_FAILURE
};      да се имплементира система за приемане на информация за грешки при електрическите консуматори */


struct __attribute__((packed))myPacket {
  uint32_t boatID;
  uint8_t moduleID;
  uint8_t konsumator;
  uint8_t command;
};


const struct myPacket ZERO_PACKET = {0};

void resetPacket(struct myPacket *p)
{
    if (p == NULL) {
        return;
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

struct myPacket feedbackBuffer[FBBUFFSIZE];
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
    previousMillis[index] = millis();
    
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
  int packetSize = LoRa.parsePacket();
  if(packetSize == sizeof(myPacket)) {
      LoRa.readBytes((uint8_t*)&myData, sizeof(myData));
  }
  unsigned long currentMillis = millis();

  for(int i=0; i<FBBUFFSIZE; i++){
    if(memcmp(&feedbackBuffer[i], &myData, sizeof(struct myPacket)) == 0){
       resetPacket(&feedbackBuffer[i]);
       feedbackRecieved[i] = 1;
       previousMillis[i] = 0;
       bufferRetries[i] = 0;
    }

    if (bufferRetries[i] >= MAXRETRIES){
       handleError("Error in radio communication");
       resetPacket(&feedbackBuffer[i]);
       feedbackRecieved[i] = 1;
       previousMillis[i] = 0;
       bufferRetries[i] = 0;
    }

    if(feedbackRecieved[i] != 1 && bufferRetries[i]<5){
      if (currentMillis - previousMillis[i] >= interval) {
        previousMillis[i] = currentMillis + random(0, 150);
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&feedbackBuffer[i], sizeof(feedbackBuffer[i]));
        LoRa.endPacket();
        bufferRetries[i]+=1;
      }
    }
  }
}

volatile bool ev_hodovi = false;
volatile bool ev_sirena = false;
volatile bool ev_zadnaP = false;
volatile bool ev_prednaP = false;

volatile bool ev_rudan = false;
volatile bool ev_rudan_allow = false;

void isr_hodovi() {
  ev_hodovi = true;
}

void isr_sirena() {
  ev_sirena = true;
}

void isr_zadnaP() {
  ev_zadnaP = true;
}

void isr_prednaP() {
  ev_prednaP = true;
}

void isr_rudan() {
  ev_rudan = true;
}
bool allow_rudan = 0;
void isr_rudan_allow() {
  ev_rudan_allow = true;
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
  if (ev_hodovi) {
    ev_hodovi = false;
  
    if (digitalRead(hod_svetl) == LOW)
      sendCommand(zaden, hodovi, ON);
    else
      sendCommand(zaden, hodovi, OFF);
  }
  
  if (ev_sirena) {
    ev_sirena = false;
  
    if (digitalRead(sirenaa) == LOW)
      sendCommand(zaden, sirena, ON);
    else
      sendCommand(zaden, sirena, OFF);
  }
  
  if (ev_zadnaP) {
    ev_zadnaP = false;
  
    if (digitalRead(zadna_p) == LOW)
      sendCommand(zaden, zadnaP, ON);
    else
      sendCommand(zaden, zadnaP, OFF);
  }
  
  if (ev_prednaP) {
    ev_prednaP = false;
  
    if (digitalRead(predna_p) == LOW)
      sendCommand(preden, prednaP, ON);
    else
      sendCommand(preden, prednaP, OFF);
  }
  
  if (ev_rudan_allow) {
    ev_rudan_allow = false;
  
    allow_rudan = (digitalRead(rudan_allow) == LOW);
    if(!allow_rudan)sendCommand(preden, rudan, OFF);
  }
  
  if (ev_rudan) {
    ev_rudan = false;
  
    if (digitalRead(rudan) == LOW && allow_rudan) {
      sendCommand(preden, rudan, ON);
    } else {
      sendCommand(preden, rudan, OFF);
    }
  }
  switch(currentState) {
    case MYIDLE:
      handleIdle();
      break;
    case AWAITFEEDBACK:
      awaitFeedback(feedbackBuffer);
      break;
    default:
      break;
  }
}
