#include <Arduino.h>
#include <LoRa.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ── Log buffer ────────────────────────────────────────────────────────────
#define LOG_SIZE 80
#define LOG_MSG_LEN 150

struct LogEntry {
  unsigned long ms;
  char msg[LOG_MSG_LEN];
};

LogEntry logBuffer[LOG_SIZE];
uint8_t logHead = 0, logTail = 0, logCount = 0;

// ── LoRa пинове ──────────────────────────────────────────────────────────
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_SS    18
#define LORA_RST   14
#define LORA_DIO0  26

// ── WiFi AP ───────────────────────────────────────────────────────────────
const char* AP_SSID = "Korab-Kontrol";
const char* AP_PASS = "12345678";   // мин. 8 символа за WPA2

// ── Протокол ──────────────────────────────────────────────────────────────
#define BOATID       0x87878787
#define FBBUFFSIZE   8
#define RXBUFFSIZE   8
#define MAXRETRIES   5
#define TX_DELAY_MS  500

enum modules     { preden = 1, zaden };
enum moduleStatus { working = 1, communication_error };
enum konsumatori { hodovi = 1, sirena, zadnaP, prednaP, rudan };
enum commands    { OFF = 0, ON = 1 };

struct __attribute__((packed)) myPacket {
  uint32_t boatID;
  uint8_t  moduleID;
  uint8_t  konsumator;
  uint8_t  command;
};

const myPacket ZERO_PACKET = {0};

// ── Статус ────────────────────────────────────────────────────────────────
struct {
  uint8_t preden = working;
  uint8_t zaden = working;
} modStatus;

// Потвърдено (ACK-нато) състояние — индекс = konsumator (1..5)
uint8_t consumerState[6] = {0};

// ── TX опашка ─────────────────────────────────────────────────────────────
myPacket txQueue[FBBUFFSIZE];
uint8_t  txHead = 0, txTail = 0, txCount = 0;

// ── Feedback буфер ────────────────────────────────────────────────────────
myPacket      feedbackBuffer[FBBUFFSIZE];
unsigned long fbTimestamp[FBBUFFSIZE] = {0};
uint8_t       fbRetries[FBBUFFSIZE]   = {0};
const long    retryInterval = 2000;

bool txPush(myPacket p) {
  // Анти-спам: провери дали същата команда вече е в опашката
  for (uint8_t i = 0; i < txCount; i++) {
    uint8_t idx = (txHead + i) % FBBUFFSIZE;
    if (txQueue[idx].moduleID   == p.moduleID &&
        txQueue[idx].konsumator == p.konsumator &&
        txQueue[idx].command    == p.command) {
      Serial.println("[SKIP] Дублирана команда — игнорирана.");
      logAdd("[WARN] Дублирана команда е блокирана (анти-спам).");
      return true; // връщаме true за да не вдигаме грешка в UI-a
    }
  }
  for (int i = 0; i < FBBUFFSIZE; i++) {
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(myPacket)) != 0 &&
        feedbackBuffer[i].moduleID   == p.moduleID &&
        feedbackBuffer[i].konsumator == p.konsumator &&
        feedbackBuffer[i].command    == p.command) {
      Serial.println("[SKIP] Командата е в изчакване на обратна връзка — игнорирана.");
      logAdd("[WARN] Дублирана команда блокирана (очаква обратна връзка).");
      return true;
    }
  }
  if (txCount >= FBBUFFSIZE) {
    Serial.println("[ERR] TX queue full");
    logAdd("[WARN] TX опашката е пълна — пакетът е отхвърлен.");  // ← ADD
    return false;
  }
  txQueue[txTail] = p;  txTail = (txTail + 1) % FBBUFFSIZE;  txCount++;  return true;
}
bool txPop(myPacket *out) {
  if (txCount == 0) return false;
  *out = txQueue[txHead];  txHead = (txHead + 1) % FBBUFFSIZE;  txCount--;  return true;
}



int  fbFindEmpty() {
  for (int i = 0; i < FBBUFFSIZE; i++)
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(myPacket)) == 0) return i;
  return -1;
}
void fbAdd(myPacket p) {
  int idx = fbFindEmpty();
  if (idx == -1) {
    Serial.println("[ERR] feedback buffer full");
    logAdd("[WARN] Feedback буферът е пълен — пакетът е изгубен.");  // ← ADD
    return;
  }
  feedbackBuffer[idx] = p;  fbTimestamp[idx] = millis();  fbRetries[idx] = 0;
}
void fbClear(int idx) {
  memset(&feedbackBuffer[idx], 0, sizeof(myPacket));
  fbTimestamp[idx] = 0;  fbRetries[idx] = 0;
}

// ── RX буфер ──────────────────────────────────────────────────────────────
myPacket rxBuffer[RXBUFFSIZE];
uint8_t  rxHead = 0, rxTail = 0, rxCount = 0;

bool rxPush(myPacket p) {
  if (rxCount >= RXBUFFSIZE) {
    Serial.println("[ERR] RX buffer full");
    logAdd("[WARN] RX буферът е пълен — входящ пакет е изгубен.");  // ← ADD
    return false;
  }
  rxBuffer[rxTail] = p;  rxTail = (rxTail + 1) % RXBUFFSIZE;  rxCount++;  return true;
}
bool rxFind(myPacket expected) {
  for (uint8_t i = 0; i < rxCount; i++) {
    uint8_t ri = (rxHead + i) % RXBUFFSIZE;
    if (memcmp(&rxBuffer[ri], &expected, sizeof(myPacket)) == 0) {
      for (uint8_t j = i; j < rxCount - 1; j++) {
        rxBuffer[(rxHead + j) % RXBUFFSIZE] = rxBuffer[(rxHead + j + 1) % RXBUFFSIZE];
      }
      rxCount--;
      rxTail = (rxTail == 0) ? RXBUFFSIZE - 1 : rxTail - 1;
      return true;
    }
  }
  return false;
}


// ── TX/ACK флагове ────────────────────────────────────────────────────────
bool          waitingForAck = false;
unsigned long lastTxTime    = 0;

// ── WebServer ─────────────────────────────────────────────────────────────
WebServer server(80);

void handleStatus() {
  StaticJsonDocument<256> doc;
  JsonObject cons = doc.createNestedObject("consumers");
  cons["hodovi"]  = consumerState[hodovi];
  cons["sirena"]  = consumerState[sirena];
  cons["zadnaP"]  = consumerState[zadnaP];
  cons["prednaP"] = consumerState[prednaP];
  cons["rudan"]   = consumerState[rudan];
  cons["rudan"]   = consumerState[rudan];
  JsonObject mods = doc.createNestedObject("modules");
  mods["zaden"]  = (modStatus.zaden  == working) ? "working" : "communication_error";
  mods["preden"] = (modStatus.preden == working) ? "working" : "communication_error";
  String out;  serializeJson(doc, out);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", out);
}

void handleCommand() {
  if (server.method() != HTTP_POST) {
    server.send(405);
    return;
  }
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON");
    return;
  }
  myPacket p = { BOATID, (uint8_t)(int)doc["moduleID"], (uint8_t)(int)doc["konsumator"], (uint8_t)(int)doc["command"] };
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(txPush(p) ? 200 : 503, "application/json", txPush(p) ? "{\"ok\":true}" : "{\"ok\":false}");
  // Note: txPush вече е извикан горе за отговора — за да не се бута два пъти
  // по-чисто е така:
}

// По-чиста версия на handleCommand:
void handleCommandClean() {
  if (server.method() != HTTP_POST) {
    server.send(405);
    return;
  }
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, server.arg("plain"))) {
    server.send(400, "text/plain", "Bad JSON");
    return;
  }
  myPacket p = {
    BOATID,
    (uint8_t)(int)doc["moduleID"],
    (uint8_t)(int)doc["konsumator"],
    (uint8_t)(int)doc["command"]
  };
  bool ok = txPush(p);
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(ok ? 200 : 503, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"error\":\"queue full\"}");
}

// Сервира вградения HTML
void handleRoot();
void handleLogPage();

void setupWebServer() {
  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/status",  HTTP_GET,  handleStatus);
  server.on("/command", HTTP_POST, handleCommandClean);// in setupWebServer():
  server.on("/log-data",  HTTP_GET,  handleLogData);
  server.on("/log-clear", HTTP_POST, handleLogClear);
  server.on("/log",       HTTP_GET,  handleLogPage);
  server.begin();
}

// ── Radio ─────────────────────────────────────────────────────────────────
void radioReceivePoll() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == sizeof(myPacket)) {
    myPacket p;
    LoRa.readBytes((uint8_t*)&p, sizeof(p));
    rxPush(p);
  } else if (packetSize > 0) {
    while (LoRa.available()) LoRa.read();
  }
}

void txManager() {
  char _lb[LOG_MSG_LEN];


  if (waitingForAck || txCount == 0) return;
  if (millis() - lastTxTime < TX_DELAY_MS) return;
  myPacket p;
  if (!txPop(&p)) return;
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&p, sizeof(p));
  LoRa.endPacket();
  LoRa.receive();
  fbAdd(p);
  lastTxTime    = millis();
  waitingForAck = true;
  Serial.printf("[TX] mod=%u kons=%u cmd=%u\n", p.moduleID, p.konsumator, p.command);
  const char* modName_tx  = (p.moduleID == preden) ? "Преден модул" : "Заден модул";
  const char* konsName_tx;
  if      (p.konsumator == hodovi)  konsName_tx = "Ходови светлини";
  else if (p.konsumator == sirena)  konsName_tx = "Сирена";
  else if (p.konsumator == zadnaP)  konsName_tx = "Задна помпа";
  else if (p.konsumator == prednaP) konsName_tx = "Предна помпа";
  else if (p.konsumator == rudan)   konsName_tx = "Рудан";
  else                              konsName_tx = "Неизвестен";
  const char* cmdName_tx = (p.command == ON) ? "Включи" : "Изключи";
  snprintf(_lb, sizeof(_lb), "[TX] -> %s / %s / %s", modName_tx, konsName_tx, cmdName_tx);
  logAdd(_lb);
}

void feedbackManager() {
  //char _lb[LOG_MSG_LEN];
  unsigned long now = millis();
  bool anyPending   = false;
  for (int i = 0; i < FBBUFFSIZE; i++) {
    if (memcmp(&feedbackBuffer[i], &ZERO_PACKET, sizeof(myPacket)) == 0) continue;
    anyPending = true;
    if (rxFind(feedbackBuffer[i])) {
      uint8_t k = feedbackBuffer[i].konsumator;
      if (k >= 1 && k <= 5) consumerState[k] = feedbackBuffer[i].command;
      if (feedbackBuffer[i].moduleID == preden) modStatus.preden = working;
      else                                       modStatus.zaden  = working;
      // ── ADD ──────────────────────────────────────────────────────────
      char _lb[LOG_MSG_LEN];
      if (fbRetries[i] != 0) {
        const char* modName_ack  = (feedbackBuffer[i].moduleID == preden) ? "Преден модул" : "Заден модул";
    const char* konsName_ack;
    if      (feedbackBuffer[i].konsumator == hodovi)  konsName_ack = "Ходови светлини";
    else if (feedbackBuffer[i].konsumator == sirena)  konsName_ack = "Сирена";
    else if (feedbackBuffer[i].konsumator == zadnaP)  konsName_ack = "Задна помпа";
    else if (feedbackBuffer[i].konsumator == prednaP) konsName_ack = "Предна помпа";
    else if (feedbackBuffer[i].konsumator == rudan)   konsName_ack = "Рудан";
    else                                              konsName_ack = "Неизвестен";
    const char* cmdName_ack = (feedbackBuffer[i].command == ON) ? "Включи" : "Изключи";
    snprintf(_lb, sizeof(_lb), "[ACK] %s / %s / %s — потвърдено след %u повторения",
         modName_ack, konsName_ack, cmdName_ack, fbRetries[i]);
        logAdd(_lb);
      } else {
    const char* modName_ack  = (feedbackBuffer[i].moduleID == preden) ? "Преден модул" : "Заден модул";
    const char* konsName_ack;
    if      (feedbackBuffer[i].konsumator == hodovi)  konsName_ack = "Ходови светлини";
    else if (feedbackBuffer[i].konsumator == sirena)  konsName_ack = "Сирена";
    else if (feedbackBuffer[i].konsumator == zadnaP)  konsName_ack = "Задна помпа";
    else if (feedbackBuffer[i].konsumator == prednaP) konsName_ack = "Предна помпа";
    else if (feedbackBuffer[i].konsumator == rudan)   konsName_ack = "Рудан";
    else                                              konsName_ack = "Неизвестен";
    const char* cmdName_ack = (feedbackBuffer[i].command == ON) ? "Включи" : "Изключи";
        snprintf(_lb, sizeof(_lb), "[ACK] %s / %s / %s — потвърдено след 0 повторения",
         modName_ack, konsName_ack, cmdName_ack);
        logAdd(_lb);
      }

      // ─────────────────────────────────────────────────────────────────
      fbClear(i);
      waitingForAck = false;
      continue;
    }
    if (now - fbTimestamp[i] >= retryInterval) {
      if (fbRetries[i] >= MAXRETRIES) {
        if (feedbackBuffer[i].moduleID == preden) modStatus.preden = communication_error;
        else                                       modStatus.zaden  = communication_error;
        Serial.printf("[ERR] No ACK mod=%u kons=%u\n", feedbackBuffer[i].moduleID, feedbackBuffer[i].konsumator);
        // ── ADD ──────────────────────────────────────────────────────────
        char _lb[LOG_MSG_LEN];
        const char* modName_err  = (feedbackBuffer[i].moduleID == preden) ? "Преден модул" : "Заден модул";
        const char* konsName_err;
        if      (feedbackBuffer[i].konsumator == hodovi)  konsName_err = "Ходови светлини";
        else if (feedbackBuffer[i].konsumator == sirena)  konsName_err = "Сирена";
        else if (feedbackBuffer[i].konsumator == zadnaP)  konsName_err = "Задна помпа";
        else if (feedbackBuffer[i].konsumator == prednaP) konsName_err = "Предна помпа";
        else if (feedbackBuffer[i].konsumator == rudan)   konsName_err = "Рудан";
        else                                              konsName_err = "Неизвестен";
        snprintf(_lb, sizeof(_lb), "[ERR] %s / %s — без ACK след %u повторения. ГРЕШКА В КОМУНИКАЦИЯТА.",
             modName_err, konsName_err, MAXRETRIES);
        logAdd(_lb);
        // ─────────────────────────────────────────────────────────────────
        fbClear(i);
        waitingForAck = false;
      } else {
        fbTimestamp[i] = now;// + 1000 + random(0, 150);
        fbRetries[i]++;
        LoRa.beginPacket();
        LoRa.write((uint8_t*)&feedbackBuffer[i], sizeof(feedbackBuffer[i]));
        LoRa.endPacket();
        LoRa.receive();
        Serial.printf("[RETRY %u]\n", fbRetries[i]);
        // ── ADD ──────────────────────────────────────────────────────────
        char _lb[LOG_MSG_LEN];
        const char* modName_ret  = (feedbackBuffer[i].moduleID == preden) ? "Преден модул" : "Заден модул";
        const char* konsName_ret;
        if      (feedbackBuffer[i].konsumator == hodovi)  konsName_ret = "Ходови светлини";
        else if (feedbackBuffer[i].konsumator == sirena)  konsName_ret = "Сирена";
        else if (feedbackBuffer[i].konsumator == zadnaP)  konsName_ret = "Задна помпа";
        else if (feedbackBuffer[i].konsumator == prednaP) konsName_ret = "Предна помпа";
        else if (feedbackBuffer[i].konsumator == rudan)   konsName_ret = "Рудан";
        else                                              konsName_ret = "Неизвестен";
        snprintf(_lb, sizeof(_lb), "[RETRY] %s / %s — повторение #%u / %u",
            modName_ret, konsName_ret, fbRetries[i], MAXRETRIES);
        logAdd(_lb); 
        // ─────────────────────────────────────────────────────────────────
      }
    }
  }
  if (!anyPending) waitingForAck = false;
}

void sendCommand(uint8_t moduleID, uint8_t kons, uint8_t cmd) {
  myPacket p = {BOATID, moduleID, kons, cmd};
  txPush(p);
}

// ── Log buffer ────────────────────────────────────────────────────────────

void logAdd(const char* msg) {
  logBuffer[logTail].ms = millis();
  strncpy(logBuffer[logTail].msg, msg, LOG_MSG_LEN - 1);
  logBuffer[logTail].msg[LOG_MSG_LEN - 1] = '\0';
  logTail = (logTail + 1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
  else logHead = (logHead + 1) % LOG_SIZE;
  Serial.println(msg);  // mirrors to Serial as well
}

void handleLogData() {
  String json = "[";
  for (uint8_t i = 0; i < logCount; i++) {
    uint8_t idx = (logHead + i) % LOG_SIZE;
    if (i > 0) json += ",";
    json += "{\"ms\":";
    json += logBuffer[idx].ms;
    json += ",\"msg\":\"";
    // escape quotes in msg
    for (char* p = logBuffer[idx].msg; *p; p++) {
      if (*p == '"') json += "\\\"";
      else json += *p;
    }
    json += "\"}";
  }
  json += "]";
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", json);
}

void handleLogClear() {
  logHead = logTail = logCount = 0;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send(200, "application/json", "{\"ok\":true}");
}


// ── Setup / Loop ──────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());   // обикновено 192.168.4.1

  setupWebServer();
  Serial.println("HTTP server started — отвори 192.168.4.1 в браузъра");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("[ERR] LoRa не стартира!");
    logAdd("[ERR] LoRa не стартира! Проверете хардуера.");  // ← ADD
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  LoRa.receive();

  logAdd("[SYS] Системата стартира. LoRa OK. Изпращане на OFF до всички консуматори.");

  // Изпрати OFF на всички консуматори при старт
  sendCommand(zaden,  hodovi,  OFF);
  sendCommand(zaden,  sirena,  OFF);
  sendCommand(zaden,  zadnaP,  OFF);
  sendCommand(preden, prednaP, OFF);
  sendCommand(preden, rudan,   OFF);
}

void loop() {
  server.handleClient();
  radioReceivePoll();
  feedbackManager();
  txManager();
}




const char INDEX_HTML[] PROGMEM = R"rawhtml(

<!DOCTYPE html>
<html data-theme="dark">
<head>
<meta charset="utf-8">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@600;700&family=IBM+Plex+Sans&display=swap" rel="stylesheet">
<style>
:root{
  --bg:#f5f4f0;--surface:#ffffff;--border:#d0cec8;--text:#1a1916;--text-soft:#6b6860;
  --on-bg:#1a1916;--on-text:#f5f4f0;--off-bg:#2f2e28;--off-text:#a09a8c;
  --warn:rgba(230,140,30,0.18);--warn-border:#e68c1e;--ok:#2a7a4b;--err:#c0410e;
  --radius:8px;
  --dash-color:#b0aea8;
}
[data-theme="dark"]{
  --bg:#12110e;--surface:#1e1d1a;--border:#38362f;--text:#e8e6df;--text-soft:#888070;
  --on-bg:#e8e6df;--on-text:#12110e;--off-bg:#2a2924;--off-text:#888070;
  --warn:rgba(230,140,30,0.13);--warn-border:#b06a10;--ok:#3daa6a;--err:#e05a2a;
  --dash-color:#4a4840;
}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'IBM Plex Sans',sans-serif;background:var(--bg);color:var(--text);min-height:100vh;padding:16px;transition:background .4s,color .4s;position:relative;overflow-x:hidden}
#bg-canvas{position:fixed;inset:0;width:100%;height:100%;pointer-events:none;z-index:0}

.wrap{position:relative;z-index:1;max-width:820px;margin:0 auto;display:flex;flex-direction:column;gap:14px}

.main-grid{
  display:grid;
  grid-template-columns:2fr 1fr;
  gap:14px;
  align-items:stretch;
}

.left-col{
  display:flex;
  flex-direction:column;
  gap:14px;
}

/* DASHED GROUP — horizontal row of buttons */
.group{
  border:2px dashed var(--dash-color);
  border-radius:12px;
  padding:10px;
  display:grid;
  grid-template-columns:1fr 1fr;
  gap:10px;
}

/* RUDAN group — full height */
.group-rudan{
  border:2px dashed var(--dash-color);
  border-radius:12px;
  padding:10px;
  display:flex;
  flex-direction:column;
  height:100%;
}

/* Regular buttons */
.big-btn{
  font-family:'IBM Plex Mono',monospace;
  font-weight:700;
  letter-spacing:.04em;
  border-radius:var(--radius);
  background:#3a3832;        /* светло тъмно сиво */
  color:#f1efe8;             /* мек светъл текст */
  border:2px solid #5a574f;  /* по-светла рамка */
  box-shadow:0 2px 6px rgba(0,0,0,0.25);

  cursor:pointer;
  transition:background .2s,color .2s,border-color .2s,transform .1s, box-shadow .2s;
  user-select:none;
  width:100%;
  height:155px;
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  gap:10px;
  text-transform:uppercase;
  padding:12px 8px;

  /* optional depth */
  box-shadow:0 2px 6px rgba(0,0,0,0.25);
}

.big-btn .btn-name{
  font-size:.82rem;
  font-weight:700;
  letter-spacing:.08em;
  text-align:center;
  line-height:1.4;
  opacity:.85;
}

.big-btn .btn-status{
  font-size:1.15rem;
  font-weight:700;
  display:flex;
  align-items:center;
  gap:8px;
}

.big-btn .dot{
  width:11px;
  height:11px;
  border-radius:50%;

  /* ✅ darker visible dot when OFF */
  background:#8c877c; 

  flex-shrink:0;
  transition:background .2s,box-shadow .2s;
}

/* ✅ ON state stays strong */
.big-btn.on{
  background:var(--on-bg);
  color:var(--on-text);
  border-color:var(--on-bg);
}

.big-btn.on .dot{
  background:#3daa6a;
  box-shadow:0 0 6px #3daa6a;
}

.big-btn:active{
  transform:scale(.96);
}

.big-btn:disabled{
  opacity:.4;
  cursor:not-allowed;
  transform:none;
}

/* RUDAN */
.hold-btn{
  font-family:'IBM Plex Mono',monospace;
  font-weight:700;
  letter-spacing:.05em;
  border-radius:var(--radius);
  background:#3a3832;        /* същия тон за консистентност */
  color:#f1efe8;
  border:2px solid #5a574f;
  box-shadow:0 2px 6px rgba(0,0,0,0.25);

  cursor:pointer;
  user-select:none;
  -webkit-user-select:none;
  transition:background .2s,color .2s,border-color .2s;
  position:relative;
  overflow:hidden;
  display:flex;
  flex-direction:column;
  align-items:center;
  justify-content:center;
  gap:12px;
  width:100%;
  flex:1;
  padding:14px 10px;
  text-transform:uppercase;
  text-align:center;

  box-shadow:0 2px 6px rgba(0,0,0,0.25);
}

.hold-btn .btn-name{
  font-size:.82rem;
  font-weight:700;
  letter-spacing:.1em;
  opacity:.85;
  text-align:center;
  line-height:1.4;
}

.hold-btn .btn-status{
  font-size:.85rem;
  font-weight:700;
  text-align:center;
  line-height:1.7;
}

.hold-btn.arming{
  border-color:#e68c1e;
  color:#e68c1e;
}

.hold-btn.active{
  background:rgba(61,170,106,0.10);
  border-color:#3daa6a;
  animation:rudanGlow 2s ease-in-out infinite;
}

@keyframes rudanGlow{
  0%,100%{
    box-shadow:0 0 18px rgba(61,170,106,0.25), inset 0 0 18px rgba(61,170,106,0.12);
  }
  50%{
    box-shadow:0 0 28px rgba(61,170,106,0.45), inset 0 0 22px rgba(61,170,106,0.18);
  }
}

.hold-progress{
  position:absolute;
  left:0;
  right:0;
  bottom:0;
  height:0%;
  width:100%;
  background:rgba(230,140,30,.22);
  transition:height 3s linear;
}

.hold-btn.arming .hold-progress{
  height:100%;
}

/* HEADER at bottom */
.header{display:flex;align-items:center;gap:12px;width:100%}
.theme-pill{
  background:var(--surface);border:1.5px solid var(--border);border-radius:20px;
  padding:7px 16px;cursor:pointer;font-family:'IBM Plex Mono',monospace;font-size:.7rem;
  font-weight:600;letter-spacing:.08em;color:var(--text-soft);display:flex;align-items:center;
  gap:6px;transition:background .3s,border-color .3s;user-select:none;white-space:nowrap;flex-shrink:0;
}
.theme-pill:hover{border-color:var(--text-soft)}
.spacer{flex:1}
.mod-indicators{display:flex;gap:10px;flex-shrink:0}
.mod-ind{display:flex;align-items:center;gap:8px;background:var(--surface);border:1.5px solid var(--border);border-radius:20px;padding:7px 14px;transition:background .3s,border-color .3s}
.mod-ind.warn{background:var(--warn);border-color:var(--warn-border)}
.mod-ind-label{font-family:'IBM Plex Mono',monospace;font-size:.65rem;font-weight:600;letter-spacing:.07em;text-transform:uppercase;color:var(--text-soft);white-space:nowrap}
.led{width:11px;height:11px;border-radius:50%;flex-shrink:0;transition:background .3s,box-shadow .3s}
.led.ok{background:var(--ok);box-shadow:0 0 5px var(--ok)}
.led.err{background:var(--err);box-shadow:0 0 5px var(--err)}
</style>
</head>
<body>

<canvas id="bg-canvas"></canvas>

<div class="wrap">
  <div class="main-grid">

    <div class="left-col">
      <!-- GROUP 1: Сирена + Ходови светлини — хоризонтално с пунктир -->
      <div class="group">
        <button class="big-btn" id="btn-sirena" onclick="toggle(2,2,'sirena')">
          <span class="btn-name">Сирена</span>
          <span class="btn-status"><div class="dot"></div>OFF</span>
        </button>
        <button class="big-btn" id="btn-hodovi" onclick="toggle(2,1,'hodovi')">
          <span class="btn-name">Ходови светлини</span>
          <span class="btn-status"><div class="dot"></div>OFF</span>
        </button>
      </div>

      <!-- GROUP 2: Предна + Задна помпа — хоризонтално с пунктир -->
      <div class="group">
        <button class="big-btn" id="btn-prednaP" onclick="toggle(1,4,'prednaP')">
          <span class="btn-name">Предна помпа</span>
          <span class="btn-status"><div class="dot"></div>OFF</span>
        </button>
        <button class="big-btn" id="btn-zadnaP" onclick="toggle(2,3,'zadnaP')">
          <span class="btn-name">Задна помпа</span>
          <span class="btn-status"><div class="dot"></div>OFF</span>
        </button>
      </div>
    </div>

    <!-- GROUP 3: Рудан — full height с пунктир -->
    <div class="group-rudan">
      <button class="hold-btn" id="btn-rudan"
        onmousedown="rudanDown()" onmouseup="rudanUp()" onmouseleave="rudanUp()"
        ontouchstart="rudanDown(event)" ontouchend="rudanUp()" ontouchcancel="rudanUp()">
        <div class="hold-progress" id="rudan-progress"></div>
        <span class="btn-name">Рудан</span>
        <span class="btn-status" id="rudan-status">ЗАДРЪЖ<br>&gt; 3 СЕК.<br>ЗА<br>АКТИВИРАНЕ</span>
      </button>
    </div>

  </div>

  <!-- HEADER at bottom -->
  <div class="header">
    <a href="/log" class="theme-pill">&#9776; LOG</a>
    <div class="spacer"></div>
    <div class="mod-indicators">
      <div class="mod-ind" id="mi-preden">
        <span class="mod-ind-label">Преден модул</span>
        <div class="led ok" id="led-preden"></div>
      </div>
      <div class="mod-ind" id="mi-zaden">
        <span class="mod-ind-label">Заден модул</span>
        <div class="led ok" id="led-zaden"></div>
      </div>
    </div>
  </div>
</div>

<script>
(function(){
  const cv=document.getElementById('bg-canvas');
  const ctx=cv.getContext('2d');
  let W,H,pts,mouse={x:-999,y:-999};
  function resize(){W=cv.width=innerWidth;H=cv.height=innerHeight;}
  function dark(){return document.documentElement.getAttribute('data-theme')==='dark';}
  function colors(){
    return dark()
      ?{p:'rgba(61,170,106,',l:'rgba(61,170,106,',w:['rgba(61,170,106,0.05)','rgba(232,230,223,0.025)','rgba(61,170,106,0.035)']}
      :{p:'rgba(42,122,75,',l:'rgba(42,122,75,',w:['rgba(42,122,75,0.06)','rgba(26,25,22,0.03)','rgba(42,122,75,0.04)']};
  }
  function initPts(){pts=Array.from({length:52},()=>({x:Math.random()*W,y:Math.random()*H,vx:(Math.random()-.5)*.3,vy:(Math.random()-.5)*.3,r:1+Math.random()*1.6,a:.15+Math.random()*.4}));}
  const waves=[{fy:.30,amp:26,freq:.006,speed:.00075,phase:0},{fy:.56,amp:18,freq:.009,speed:.0005,phase:1.1},{fy:.76,amp:14,freq:.007,speed:.001,phase:2.4}];
  function frame(){
    ctx.clearRect(0,0,W,H);const C=colors();
    waves.forEach((w,i)=>{w.phase+=w.speed;ctx.beginPath();for(let x=0;x<=W;x+=4){const y=H*w.fy+Math.sin(x*w.freq+w.phase)*w.amp;x===0?ctx.moveTo(x,y):ctx.lineTo(x,y);}ctx.lineTo(W,H);ctx.lineTo(0,H);ctx.closePath();ctx.fillStyle=C.w[i];ctx.fill();});
    for(let i=0;i<pts.length;i++)for(let j=i+1;j<pts.length;j++){const dx=pts[i].x-pts[j].x,dy=pts[i].y-pts[j].y,d=Math.hypot(dx,dy);if(d<115){ctx.beginPath();ctx.moveTo(pts[i].x,pts[i].y);ctx.lineTo(pts[j].x,pts[j].y);ctx.strokeStyle=C.l+((1-d/115)*.15)+')';ctx.lineWidth=.7;ctx.stroke();}}
    pts.forEach(p=>{const dx=p.x-mouse.x,dy=p.y-mouse.y,d=Math.hypot(dx,dy);if(d<85&&d>0){const f=(85-d)/85;p.vx+=dx/d*f*.1;p.vy+=dy/d*f*.1;}const sp=Math.hypot(p.vx,p.vy);if(sp>1.3){p.vx=p.vx/sp*1.3;p.vy=p.vy/sp*1.3;}p.x+=p.vx;p.y+=p.vy;if(p.x<0){p.x=0;p.vx*=-1;}if(p.x>W){p.x=W;p.vx*=-1;}if(p.y<0){p.y=0;p.vy*=-1;}if(p.y>H){p.y=H;p.vy*=-1;}ctx.beginPath();ctx.arc(p.x,p.y,p.r,0,Math.PI*2);ctx.fillStyle=C.p+p.a+')';ctx.fill();});
    requestAnimationFrame(frame);
  }
  addEventListener('mousemove',e=>{mouse.x=e.clientX;mouse.y=e.clientY;});
  addEventListener('touchmove',e=>{if(e.touches[0]){mouse.x=e.touches[0].clientX;mouse.y=e.touches[0].clientY;}},{passive:true});
  addEventListener('resize',()=>{resize();initPts();});
  resize();initPts();frame();
})();

function toggleTheme(){
  const html=document.documentElement;
  const dark=html.getAttribute('data-theme')==='dark';
  html.setAttribute('data-theme',dark?'light':'dark');
  document.getElementById('theme-icon').innerHTML=dark?'&#9788;':'&#9790;';
  document.getElementById('theme-label').textContent=dark?'LIGHT':'DARK';
}

const state={hodovi:0,sirena:0,zadnaP:0,prednaP:0,modZaden:'working',modPreden:'working'};

function render(){
  ['hodovi','sirena','zadnaP','prednaP'].forEach(k=>{
  // Рудан статус след ACK
  const rudanConfirmed = state['rudan'] === 1;
  const rudanBtn = document.getElementById('btn-rudan');
  const rudanStatusEl = document.getElementById('rudan-status');
  if (rudanConfirmed && !rudanOn) {
    // не правим нищо — управлява се от hold логиката
  } else if (rudanConfirmed) {
    rudanBtn.className = 'hold-btn active';
    rudanStatusEl.innerHTML = '▶ РУДАН<br>АКТИВЕН';
  } else if (!rudanOn) {
    // вече изключен и потвърден
    rudanStatusEl.innerHTML = 'ЗАДРЪЖ<br>&gt; 3 СЕК.<br>ЗА<br>АКТИВИРАНЕ';
  }
    const on=state[k]===1;
    const btn=document.getElementById('btn-'+k);
    if(!btn)return;
    btn.classList.toggle('on',on);
    const statusEl=btn.querySelector('.btn-status');
    const dot=document.createElement('div');dot.className='dot';
    statusEl.innerHTML='';
    statusEl.appendChild(dot);
    statusEl.appendChild(document.createTextNode(on?' ON':' OFF'));
  });
  setModStatus('zaden',state.modZaden);
  setModStatus('preden',state.modPreden);
}

function setModStatus(mod,status){
  const isErr=status!=='working';
  document.getElementById('led-'+mod).className='led '+(isErr?'err':'ok');
  document.getElementById('mi-'+mod).className='mod-ind'+(isErr?' warn':'');
}

async function toggle(moduleID,konsumator,key){
  const newCmd=state[key]===1?0:1;
  const btn=document.getElementById('btn-'+key);
  btn.disabled=true;
  try{
    const res=await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({moduleID,konsumator,command:newCmd})});
    if(!res.ok)throw new Error();
  }catch(e){}
  finally{setTimeout(()=>{btn.disabled=false;},800);}
}

let rudanArmTimer=null,rudanReleaseTimer=null,rudanOn=false;

function rudanDown(e){
  if(e)e.preventDefault();
  clearTimeout(rudanReleaseTimer);
  const btn=document.getElementById('btn-rudan');
  const prog=document.getElementById('rudan-progress');
  prog.style.height='0%';
  if(!rudanOn){
    btn.className='hold-btn arming';
    rudanArmTimer=setTimeout(()=>{
      rudanOn=true;
      btn.className='hold-btn active';
      document.getElementById('rudan-status').innerHTML='▶ РУДАН<br>АКТИВЕН';
      sendRudan(1);
    },3000);
  }else{
    btn.className='hold-btn active';
  }
}

function rudanUp(){
  clearTimeout(rudanArmTimer);
  const btn=document.getElementById('btn-rudan');
  const prog=document.getElementById('rudan-progress');
  prog.style.height='0%';
  if(rudanOn){
    btn.className='hold-btn active';
    document.getElementById('rudan-status').innerHTML='ПУСКАНЕ<br>СЛЕД 0.5 СЕК…';
    rudanReleaseTimer=setTimeout(()=>{
      rudanOn=false;
      btn.className='hold-btn';
      document.getElementById('rudan-status').innerHTML='ЗАДРЪЖ<br>&gt; 3 СЕК.<br>ЗА<br>АКТИВИРАНЕ';
      sendRudan(0);
    },500);
  }else{
    btn.className='hold-btn';
    document.getElementById('rudan-status').innerHTML='ЗАДРЪЖ<br>&gt; 3 СЕК.<br>ЗА<br>АКТИВИРАНЕ';
  }
}

async function sendRudan(cmd){
  try{await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({moduleID:1,konsumator:5,command:cmd})});}catch(e){}
}

async function pollStatus(){
  try{
    const res=await fetch('/status');
    if(!res.ok)throw new Error();
    const data=await res.json();
    Object.assign(state,data.consumers);
    state.modZaden=data.modules.zaden;
    state.modPreden=data.modules.preden;
    render();
  }catch(e){}
}

render();
pollStatus();
setInterval(pollStatus,1500);
</script>
</body>
</html>


)rawhtml";

const char LOG_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html data-theme="dark">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<link href="https://fonts.googleapis.com/css2?family=IBM+Plex+Mono:wght@400;600;700&display=swap" rel="stylesheet">
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'IBM Plex Mono',monospace;background:#12110e;color:#e8e6df;min-height:100vh;padding:16px}
.header{display:flex;align-items:center;gap:10px;margin-bottom:14px;border-bottom:1px solid #2a2924;padding-bottom:12px;flex-wrap:wrap}
.back-btn{background:#1e1d1a;border:1.5px solid #38362f;border-radius:8px;color:#888070;padding:7px 16px;font-family:'IBM Plex Mono',monospace;font-size:.7rem;font-weight:600;letter-spacing:.08em;cursor:pointer;text-decoration:none;display:inline-flex;align-items:center;gap:6px}
.back-btn:hover{border-color:#888070}
h1{font-size:.85rem;font-weight:700;letter-spacing:.12em;color:#e8e6df;text-transform:uppercase}
.badge{font-size:.65rem;background:#1e1d1a;border:1.5px solid #38362f;border-radius:20px;padding:4px 10px;color:#888070;letter-spacing:.06em}
.spacer{flex:1}
.clear-btn{background:transparent;border:1.5px solid #38362f;border-radius:8px;color:#888070;padding:7px 14px;font-family:'IBM Plex Mono',monospace;font-size:.65rem;font-weight:600;letter-spacing:.08em;cursor:pointer}
.clear-btn:hover{border-color:#c0410e;color:#c0410e}
.pause-btn{background:transparent;border:1.5px solid #38362f;border-radius:8px;color:#888070;padding:7px 14px;font-family:'IBM Plex Mono',monospace;font-size:.65rem;font-weight:600;letter-spacing:.08em;cursor:pointer}
.pause-btn.active{border-color:#e68c1e;color:#e68c1e}
.log-wrap{background:#0d0c0a;border:1.5px solid #1e1d1a;border-radius:10px;padding:10px;height:calc(100vh - 160px);overflow-y:auto;display:flex;flex-direction:column;gap:3px;font-size:.72rem;line-height:1.6}
.log-wrap::-webkit-scrollbar{width:6px}
.log-wrap::-webkit-scrollbar-track{background:#0d0c0a}
.log-wrap::-webkit-scrollbar-thumb{background:#2a2924;border-radius:3px}
.entry{display:grid;grid-template-columns:90px 60px 1fr;gap:8px;padding:4px 8px;border-radius:4px}
.entry:hover{background:#16150f}
.entry.err-row{background:#1c0f0a}
.ts{color:#4a4840;font-size:.65rem;white-space:nowrap;align-self:start;padding-top:1px}
.tag{border-radius:4px;padding:1px 6px;font-size:.65rem;font-weight:600;letter-spacing:.05em;text-align:center;align-self:start;white-space:nowrap;margin-top:1px}
.TX{background:rgba(61,80,170,0.18);color:#6a84e0;border:1px solid rgba(61,80,170,0.3)}
.ACK{background:rgba(61,170,106,0.15);color:#3daa6a;border:1px solid rgba(61,170,106,0.3)}
.RETRY{background:rgba(230,140,30,0.15);color:#e68c1e;border:1px solid rgba(230,140,30,0.3)}
.ERR{background:rgba(192,65,14,0.18);color:#e05a2a;border:1px solid rgba(192,65,14,0.3)}
.SYS{background:rgba(100,100,100,0.15);color:#888070;border:1px solid rgba(100,100,100,0.2)}
.WARN{background:rgba(230,140,30,0.1);color:#d4890a;border:1px solid rgba(230,140,30,0.25)}
.msg{color:#c8c4b8;word-break:break-word}
.sep{border:none;border-top:1px solid #1a1916;margin:3px 0}
</style>
</head>
<body>
<div class="header">
  <a class="back-btn" href="/">&#8592; НАЗАД</a>
  <h1>Системен лог</h1>
  <div class="badge" id="cnt">0 записа</div>
  <div class="spacer"></div>
  <button class="clear-btn" onclick="clearLog()">ИЗЧИСТИ</button>
</div>
<div class="log-wrap" id="log"></div>
<script>
let paused=false,lastCount=0;
const TAGS=['TX','ACK','RETRY','ERR','SYS','WARN'];
function fmt(ms){const t=Math.floor(ms/1000);const h=String(Math.floor(t/3600)).padStart(2,'0');const m=String(Math.floor((t%3600)/60)).padStart(2,'0');const s=String(t%60).padStart(2,'0');const ms2=String(ms%1000).padStart(3,'0');return h+':'+m+':'+s+'.'+ms2;}
function parseTag(msg){for(const t of TAGS)if(msg.startsWith('['+t+']'))return t;return'SYS';}
function render(entries){
  const log=document.getElementById('log');
  const atBottom=log.scrollHeight-log.scrollTop<=log.clientHeight+40;
  log.innerHTML='';
  let lastSec=-1;
  entries.forEach(e=>{
    const sec=Math.floor(e.ms/1000);
    if(sec-lastSec>5&&lastSec>=0){const hr=document.createElement('hr');hr.className='sep';log.appendChild(hr);}
    lastSec=sec;
    const tag=parseTag(e.msg);
    const cleanMsg=e.msg.replace('['+tag+'] ','');
    const row=document.createElement('div');
    row.className='entry'+(tag==='ERR'?' err-row':'');
    row.innerHTML='<span class="ts">'+fmt(e.ms)+'</span><span class="tag '+tag+'">'+tag+'</span><span class="msg">'+cleanMsg+'</span>';
    log.appendChild(row);
  });
  document.getElementById('cnt').textContent=entries.length+' записа';
  if(atBottom)log.scrollTop=log.scrollHeight;
}
async function poll(){
  if(paused)return;
  try{
    const r=await fetch('/log-data');
    const data=await r.json();
    if(data.length!==lastCount){lastCount=data.length;render(data);}
  }catch(e){}
}
function togglePause(){paused=!paused;const b=document.getElementById('pause-btn');b.classList.toggle('active',paused);b.textContent=paused?'&#9654; ПРОДЪЛЖИ':'&#9646;&#9646; ПАУЗА';}
async function clearLog(){
  await fetch('/log-clear',{method:'POST'});
  lastCount=0;render([]);
}
poll();setInterval(poll,800);
</script>
</body>
</html>
)rawhtml";



void handleRoot() {
  server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

void handleLogPage() {
  server.send_P(200, "text/html; charset=utf-8", LOG_HTML);
}
