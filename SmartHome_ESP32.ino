/*
 * ============================================================
 *  SmartHome ESP32 - Sistem de alarma si control inteligent
 * ============================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_BMP085.h>
#include <LiquidCrystal.h>    
#include <ArduinoJson.h>
#include <Preferences.h>

// ============================================================
//  CONFIGURARE
// ============================================================
const char* WIFI_SSID      = "DIGI-mgY9";
const char* WIFI_PASSWORD  = "cUExzTcFn4";
const char* MQTT_SERVER    = "79a26acab6f94e7598b50404d7846773.s1.eu.hivemq.cloud";
const int   MQTT_PORT      = 8883;
const char* MQTT_USER      = "appESP";   
const char* MQTT_PASS      = "appESP32";
const char* MQTT_CLIENT_ID = "79a26acab6f94e7598b50404d7846773.s1.eu.hivemq.cloud";

// ============================================================
//  TOPICURI MQTT
// ============================================================
#define TOPIC_SENSOR_DATA  "smarthome/sensors"
#define TOPIC_RFID_NEW     "smarthome/rfid/new"
#define TOPIC_ALARM_GAS    "smarthome/alarm/gas"
#define TOPIC_ALARM_MOTION "smarthome/alarm/motion"
#define TOPIC_LOG          "smarthome/log"
#define TOPIC_STATUS       "smarthome/status"

#define TOPIC_CMD_LIGHT    "smarthome/cmd/light"
#define TOPIC_CMD_SCREEN   "smarthome/cmd/screen"
#define TOPIC_CMD_RFID_ADD "smarthome/cmd/rfid/add"
#define TOPIC_CMD_RFID_DEL "smarthome/cmd/rfid/del"
#define TOPIC_CMD_ALARM    "smarthome/cmd/alarm"
#define TOPIC_CMD_LOG_REQ  "smarthome/cmd/log"
#define TOPIC_CMD_CARD_CFG "smarthome/cmd/card/config"

// ============================================================
//  PINOUT
// ============================================================
#define PIN_MQ135      34
#define PIN_TEMT6000   35
#define PIN_PIR        36   

#define PIN_LED_R      25
#define PIN_LED_G      26
#define PIN_LED_B      16   

#define PIN_BUZZER     32
#define PIN_BUTTON     33

#define PIN_RFID_SS     5
#define PIN_RFID_RST    4

// LCD HD44780
#define PIN_LCD_RS      2
#define PIN_LCD_EN     12
#define PIN_LCD_D4     27
#define PIN_LCD_D5     14
#define PIN_LCD_D6     15
#define PIN_LCD_D7     13

// PWM LED
#define PWM_CH_R  0
#define PWM_CH_G  1
#define PWM_CH_B  2
#define PWM_FREQ  5000
#define PWM_RES   8

// ============================================================
//  CONSTANTE
// ============================================================
#define MAX_RFID_CARDS   20
#define CARD_UID_LEN      4
#define NVM_NS           "smarthome"

#define SENSOR_INTERVAL  10000UL   // ms intre citiri
#define LIGHT_THRESHOLD    512     // ADC sub = noapte 
#define GAS_THRESHOLD      1750     // ADC peste = alarma
#define GAS_CLEAR_THRESH   1200     // ADC sub = gaz a disparut
#define MOTION_SILENT_MS 60000UL   // 60s alarma silentioasa
#define LCD_SCROLL_MS     3000UL   // interval rotatie ecrane

// ============================================================
//  OBIECTE
// ============================================================
WiFiClientSecure wifiClient;
PubSubClient     mqtt(wifiClient);
MFRC522          rfid(PIN_RFID_SS, PIN_RFID_RST);
Adafruit_BMP085  bmp;
// LiquidCrystal(RS, EN, D4, D5, D6, D7)
LiquidCrystal    lcd(PIN_LCD_RS, PIN_LCD_EN,
                     PIN_LCD_D4, PIN_LCD_D5, PIN_LCD_D6, PIN_LCD_D7);
Preferences      prefs;

// ============================================================
//  STRUCTURA CARD RFID
// ============================================================
struct RFIDCard {
  uint8_t uid[CARD_UID_LEN];
  bool    valid;
  uint8_t ledR, ledG, ledB;
  bool    ledOnDay;
  bool    ledOnNight;
  bool    buzzerOnRead;
};

RFIDCard cards[MAX_RFID_CARDS];
int      cardCount = 0;

// ============================================================
//  STAREA SISTEMULUI
// ============================================================
struct Sys {
  // Alarme
  bool          alarmArmed        = false;
  bool          alarmGas          = false;
  bool          alarmMovSilent    = false;
  bool          alarmMovActive    = false;
  unsigned long movSilentStart    = 0;

  // Senzori
  float         temp    = 0, pres = 0;
  int           lightADC = 0, gasADC = 0;
  bool          isDay   = true;
  bool          pir     = false, pirPrev = false;

  // LED
  uint8_t       ledR = 255, ledG = 255, ledB = 255;
  bool          ledManual = false;

  // LCD
  bool          lcdOvr    = false;
  unsigned long lcdOvrExp = 0;
  int           lcdPage   = 0;
  unsigned long lcdLast   = 0;
  bool          lcdBlink  = false;
  unsigned long blinkLast = 0;

  // RFID
  bool          waitCard    = false;
  unsigned long waitCardExp = 0;
  int           cardIdx     = -1;

  // Timere
  unsigned long lastPub = 0;

  // Buzzer non-blocking
  bool          buzOn    = false;
  bool          buzState = false;
  unsigned long buzPer   = 500;
  unsigned long buzLast  = 0;
} S;

// ============================================================
//  LOG
// ============================================================
#define LOG_N  50
#define LOG_L  64
struct LogEntry { unsigned long ts; char msg[LOG_L]; };
LogEntry logBuf[LOG_N];
int logHead = 0, logCnt = 0;

// ============================================================
//  PROTOTIPURI
// ============================================================
void setupWiFi();
void reconnectMQTT();
void mqttCb(char*, byte*, unsigned int);
void readSensors();
void pubSensors();
void handleAlarms();
void handleLED();
void handleBuzNB();
void handleLCD();
void handleRFID();
void handleButton();
void setLED(uint8_t r, uint8_t g, uint8_t b);
void applyCardLED(int idx);
void buzStart(unsigned long per);
void buzStop();
void buzBeep(int n, int on, int off);
void lcdP(const String& a, const String& b);
void lcdOvr(const String& a, const String& b, unsigned long ms);
void saveNVM(); void loadNVM();
void addLog(const char*); void sendLog();
int  findCard(uint8_t*);
String uidS(uint8_t*, uint8_t);
bool uidEq(uint8_t*, uint8_t*);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n=== SmartHome ESP32 v2.0 ==="));

  // GPIO
  pinMode(PIN_PIR,    INPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // LED RGB PWM
  ledcAttach(PIN_LED_R, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_LED_G, PWM_FREQ, PWM_RES);
  ledcAttach(PIN_LED_B, PWM_FREQ, PWM_RES);
  setLED(0, 0, 0);

  // LCD HD44780
  lcd.begin(16, 2);
  lcdP("SmartHome v2.0", "Initializing...");

  // I2C -> BMP180
  Wire.begin(21, 22);
  if (!bmp.begin()) {
    Serial.println(F("[WARN] BMP180 negasit!"));
    lcdOvr("Eroare BMP180!", "Verif SDA/SCL", 3000);
  } else {
    Serial.println(F("[OK] BMP180"));
  }

  // SPI -> RFID
  SPI.begin(18, 19, 23, PIN_RFID_SS);
  rfid.PCD_Init();
  delay(50);
  Serial.println(F("[OK] RFID RC522"));

  // NVM
  prefs.begin(NVM_NS, false);
  loadNVM();

  // WiFi + MQTT
  setupWiFi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
  mqtt.setCallback(mqttCb);
  mqtt.setBufferSize(1024);

  addLog("Boot OK");
  lcdP("SmartHome", "Ready!");
  delay(800);
}

// ============================================================
//  LOOP
// ============================================================
void loop() {
  if (!mqtt.connected()) reconnectMQTT();
  mqtt.loop();

  unsigned long now = millis();
  if (now - S.lastPub >= SENSOR_INTERVAL) {
    S.lastPub = now;
    readSensors();
    pubSensors();
  }

  handleRFID();
  handleButton();
  handleAlarms();
  handleLED();
  handleBuzNB();
  handleLCD();
}

// ============================================================
//  WIFI
// ============================================================
void setupWiFi() {
  lcdP("Conectare WiFi:", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiClient.setInsecure();
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t++ < 40) {
    delay(500); Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\n[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    lcdOvr("WiFi OK!", WiFi.localIP().toString(), 2000);
  } else {
    Serial.println(F("\n[WiFi] Eroare! Offline."));
    lcdOvr("WiFi EROARE", "Mod offline", 3000);
  }
}

// ============================================================
//  MQTT RECONNECT
// ============================================================
void reconnectMQTT() {
  for (int i = 0; i < 3 && !mqtt.connected(); i++) {
    Serial.printf("[MQTT] Conectare #%d...\n", i + 1);
    bool ok = strlen(MQTT_USER) > 0
              ? mqtt.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASS)
              : mqtt.connect(MQTT_CLIENT_ID);
    if (ok) {
      mqtt.subscribe(TOPIC_CMD_LIGHT);
      mqtt.subscribe(TOPIC_CMD_SCREEN);
      mqtt.subscribe(TOPIC_CMD_RFID_ADD);
      mqtt.subscribe(TOPIC_CMD_RFID_DEL);
      mqtt.subscribe(TOPIC_CMD_ALARM);
      mqtt.subscribe(TOPIC_CMD_LOG_REQ);
      mqtt.subscribe(TOPIC_CMD_CARD_CFG);

      StaticJsonDocument<128> d;
      d["status"] = "online"; d["ip"] = WiFi.localIP().toString();
      d["cards"]  = cardCount;
      char b[128]; serializeJson(d, b);
      mqtt.publish(TOPIC_STATUS, b, true);
      addLog("MQTT connected");
      Serial.println(F("[MQTT] OK"));
    } else {
      Serial.printf("[MQTT] rc=%d, 5s...\n", mqtt.state());
      delay(5000);
    }
  }
}

// ============================================================
//  CALLBACK MQTT
// ============================================================
void mqttCb(char* topic, byte* payload, unsigned int len) {
  char raw[512];
  if (len >= sizeof(raw)) len = sizeof(raw) - 1;
  memcpy(raw, payload, len); raw[len] = '\0';
  Serial.printf("[MQTT<] %s : %s\n", topic, raw);

  StaticJsonDocument<512> doc;
  bool ok = !deserializeJson(doc, raw);

  // CMD LIGHT: {"r":255,"g":0,"b":0} sau {"on":false}
  if (!strcmp(topic, TOPIC_CMD_LIGHT) && ok) {
    if (doc.containsKey("r")) {
      S.ledR = (uint8_t)(int)doc["r"];
      S.ledG = (uint8_t)(int)doc["g"];
      S.ledB = (uint8_t)(int)doc["b"];
    }
    if (doc.containsKey("on") && !(bool)doc["on"]) {
      S.ledR = S.ledG = S.ledB = 0;
    }
    S.ledManual = true;
    addLog("CMD: light");
  }

  // CMD SCREEN: {"line1":"...","line2":"...","duration":5000}
  else if (!strcmp(topic, TOPIC_CMD_SCREEN) && ok) {
    lcdOvr(doc["line1"] | "SmartHome",
           doc["line2"] | "",
           (unsigned long)(int)(doc["duration"] | 5000));
  }

  // CMD RFID ADD
  else if (!strcmp(topic, TOPIC_CMD_RFID_ADD)) {
    S.waitCard    = true;
    S.waitCardExp = millis() + 30000UL;
    lcdOvr("Apropiati card", "de cititor...", 30000);
    addLog("Astept card nou");
  }

  // CMD RFID DEL
  else if (!strcmp(topic, TOPIC_CMD_RFID_DEL) && ok && doc.containsKey("uid")) {
    String uid = doc["uid"].as<String>(); uid.toUpperCase();
    for (int i = 0; i < cardCount; i++) {
      if (uidS(cards[i].uid, CARD_UID_LEN) == uid) {
        if (S.cardIdx == i) S.cardIdx = -1;
        cards[i] = cards[--cardCount];
        saveNVM();
        addLog(("Sters: " + uid).c_str());
        lcdOvr("Card sters", uid, 2000);
        StaticJsonDocument<96> d; d["event"] = "card_deleted"; d["uid"] = uid;
        char b[96]; serializeJson(d, b); mqtt.publish(TOPIC_STATUS, b);
        break;
      }
    }
  }

  // CMD ALARM
  else if (!strcmp(topic, TOPIC_CMD_ALARM) && ok && doc.containsKey("armed")) {
    S.alarmArmed = (bool)doc["armed"];
    if (!S.alarmArmed) {
      S.alarmMovSilent = S.alarmMovActive = false;
      buzStop();
    }
    addLog(S.alarmArmed ? "Alarma ARMATA (MQTT)" : "Alarma DEZARMATA (MQTT)");
    lcdOvr(S.alarmArmed ? "Alarma: ARMATA" : "Alarma: Dezarm.", "", 2000);
  }

  // CMD LOG REQ
  else if (!strcmp(topic, TOPIC_CMD_LOG_REQ)) {
    sendLog();
  }

  // CMD CARD CONFIG
  else if (!strcmp(topic, TOPIC_CMD_CARD_CFG) && ok &&
           doc.containsKey("uid") && doc.containsKey("r")) {
    String uid = doc["uid"].as<String>(); uid.toUpperCase();
    for (int i = 0; i < cardCount; i++) {
      if (uidS(cards[i].uid, CARD_UID_LEN) == uid) {
        cards[i].ledR         = (uint8_t)(int)(doc["r"]           | 255);
        cards[i].ledG         = (uint8_t)(int)(doc["g"]           | 255);
        cards[i].ledB         = (uint8_t)(int)(doc["b"]           | 255);
        cards[i].ledOnDay     = doc["ledOnDay"]     | false;
        cards[i].ledOnNight   = doc["ledOnNight"]   | true;
        cards[i].buzzerOnRead = doc["buzzerOnRead"] | true;
        saveNVM();
        addLog(("Cfg card: " + uid).c_str());
        if (S.cardIdx == i && !S.ledManual) applyCardLED(i);
        break;
      }
    }
  }
}

// ============================================================
//  SENZORI
// ============================================================
void readSensors() {
  S.temp    = bmp.readTemperature();
  S.pres    = bmp.readPressure() / 100.0f;
  S.lightADC = analogRead(PIN_TEMT6000);
  S.isDay   = (S.lightADC > LIGHT_THRESHOLD);
  S.gasADC  = analogRead(PIN_MQ135);

  // Tranzitii gaz
  if (S.gasADC > GAS_THRESHOLD && !S.alarmGas) {
    S.alarmGas = true;
    addLog("ALARMA GAZ!");
    StaticJsonDocument<96> d; d["event"]="gas_alarm"; d["adc"]=S.gasADC;
    char b[96]; serializeJson(d,b); mqtt.publish(TOPIC_ALARM_GAS, b);
  } else if (S.gasADC < GAS_CLEAR_THRESH && S.alarmGas) {
    S.alarmGas = false; buzStop();
    addLog("Gaz disparut");
    StaticJsonDocument<64> d; d["event"]="gas_clear";
    char b[64]; serializeJson(d,b); mqtt.publish(TOPIC_ALARM_GAS, b);
  }

  // PIR edge detection
  S.pir = (bool)digitalRead(PIN_PIR);
  if (S.pir && !S.pirPrev) {
    if (S.alarmArmed && !S.alarmGas && !S.alarmMovSilent && !S.alarmMovActive) {
      S.alarmMovSilent = true;
      S.movSilentStart = millis();
      addLog("Miscare: alarma silentioasa");
      StaticJsonDocument<96> d; d["event"]="motion"; d["silent"]=true;
      char b[96]; serializeJson(d,b); mqtt.publish(TOPIC_ALARM_MOTION, b);
    }
  }
  S.pirPrev = S.pir;

  Serial.printf("[S] T=%.1fC P=%.0fhPa L=%d G=%d D=%d\n",
    S.temp, S.pres, S.lightADC, S.gasADC, S.isDay);
}

void pubSensors() {
  StaticJsonDocument<256> d;
  d["temp"]=S.temp; d["pres"]=S.pres; d["light"]=S.lightADC;
  d["gas"]=S.gasADC; d["isDay"]=S.isDay;
  d["armed"]=S.alarmArmed; d["gasAlarm"]=S.alarmGas;
  d["movAlarm"]=(S.alarmMovSilent||S.alarmMovActive); d["pir"]=S.pir;
  char b[256]; serializeJson(d,b); mqtt.publish(TOPIC_SENSOR_DATA, b);
}

// ============================================================
//  ALARME
// ============================================================
void handleAlarms() {
  unsigned long now = millis();

  if (S.alarmGas) {
    if (!S.buzOn) buzStart(500);
    return;
  }

  if (S.alarmMovSilent) {
    if (S.buzOn) buzStop();
    if (now - S.movSilentStart >= MOTION_SILENT_MS) {
      S.alarmMovSilent = false;
      S.alarmMovActive = true;
      buzStart(350);
      addLog("Alarma miscare ACTIVA (buzzer+lumina)");
      StaticJsonDocument<96> d; d["event"]="motion"; d["silent"]=false;
      char b[96]; serializeJson(d,b); mqtt.publish(TOPIC_ALARM_MOTION, b);
    }
    return;
  }

  if (S.alarmMovActive) {
    if (!S.buzOn) buzStart(350);
    return;
  }

  if (S.buzOn) buzStop();
}

// ============================================================
//  LED
// ============================================================
void handleLED() {
  unsigned long now = millis();

  // P1: GAZ (700ms)
  if (S.alarmGas) {
    if (now - S.blinkLast >= 700) { S.blinkLast=now; S.lcdBlink=!S.lcdBlink; }
    setLED(S.lcdBlink ? 120 : 0, 0, 0);
    return;
  }
  // P2: Miscare activa (300ms)
  if (S.alarmMovActive) {
    if (now - S.blinkLast >= 300) { S.blinkLast=now; S.lcdBlink=!S.lcdBlink; }
    setLED(S.lcdBlink ? 255 : 0, 0, 0);
    return;
  }
  // P3: Silent 
  // P4: Override manual MQTT
  if (S.ledManual) { setLED(S.ledR, S.ledG, S.ledB); return; }
  // P5: Card activ 
  if (S.cardIdx >= 0 && S.cardIdx < cardCount) { applyCardLED(S.cardIdx); return; }
  // P6: Default zi/noapte
  setLED(S.isDay ? 0 : 255, S.isDay ? 0 : 255, S.isDay ? 0 : 255);
}

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(PIN_LED_R, r);
  ledcWrite(PIN_LED_G, g);
  ledcWrite(PIN_LED_B, b);
}

void applyCardLED(int idx) {
  RFIDCard& c = cards[idx];
  bool on = S.isDay ? c.ledOnDay : c.ledOnNight;
  setLED(on ? c.ledR : 0, on ? c.ledG : 0, on ? c.ledB : 0);
  S.ledR = on ? c.ledR : 0;
  S.ledG = on ? c.ledG : 0;
  S.ledB = on ? c.ledB : 0;
}

// ============================================================
//  BUZZER
// ============================================================
void buzStart(unsigned long per) {
  S.buzOn = true; S.buzPer = per;
}
void buzStop() {
  S.buzOn = false; S.buzState = false;
  digitalWrite(PIN_BUZZER, LOW);
}
void handleBuzNB() {
  if (!S.buzOn) return;
  unsigned long now = millis();
  if (now - S.buzLast >= S.buzPer) {
    S.buzLast = now;
    S.buzState = !S.buzState;
    digitalWrite(PIN_BUZZER, S.buzState ? HIGH : LOW);
  }
}
void buzBeep(int n, int on, int off) {
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_BUZZER, HIGH); delay(on);
    digitalWrite(PIN_BUZZER, LOW);
    if (i < n-1) delay(off);
  }
}

// ============================================================
//  LCD
// ============================================================
void lcdP(const String& a, const String& b) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(a.substring(0,16));
  lcd.setCursor(0,1); lcd.print(b.substring(0,16));
}

void lcdOvr(const String& a, const String& b, unsigned long ms) {
  S.lcdOvr    = true;
  S.lcdOvrExp = millis() + ms;
  lcdP(a, b);
}

void handleLCD() {
  unsigned long now = millis();

  if (S.lcdOvr) {
    if (now < S.lcdOvrExp) return;
    S.lcdOvr = false; lcd.clear();
  }

  // ALARMA GAZ
  if (S.alarmGas) {
    if (now - S.lcdLast >= 700) {
      S.lcdLast = now; S.lcdBlink = !S.lcdBlink;
      if (S.lcdBlink) lcdP("!! ALARMA GAZ !!", "Evacuati zona!");
      else {
        char buf[17]; snprintf(buf,17,"Gas ADC:%4d",S.gasADC);
        lcdP("!! ALARMA GAZ !!", String(buf));
      }
    }
    return;
  }

  // ALARMA MISCARE ACTIVA
  if (S.alarmMovActive) {
    if (now - S.lcdLast >= 400) {
      S.lcdLast = now; S.lcdBlink = !S.lcdBlink;
      lcdP(S.lcdBlink ? "!! ALARMA !!" : "  INTRUS  ",
           S.lcdBlink ? "MISCARE DETECT." : "Suna alarma!");
    }
    return;
  }

  // ALARMA SILENTIOASA
  if (S.alarmMovSilent) {
    if (now - S.lcdLast >= 1000) {
      S.lcdLast = now;
      unsigned long rem = (MOTION_SILENT_MS - (now - S.movSilentStart)) / 1000;
      char buf[17]; snprintf(buf,17,"Countdown: %2lus", rem);
      lcdP("Miscare detect!", String(buf));
    }
    return;
  }

  // ROTATIE NORMALA
  if (now - S.lcdLast < LCD_SCROLL_MS) return;
  S.lcdLast = now;

  char l1[17], l2[17];
  switch (S.lcdPage) {
    case 0:
      snprintf(l1,17,"T:%.1fC  P:%.0fhPa", S.temp, S.pres);
      snprintf(l2,17,"%s", S.isDay ? "Zi  - LED stins" : "Noapte - LED ON");
      break;
    case 1:
      snprintf(l1,17,"Lumina: %4d",  S.lightADC);
      snprintf(l2,17,"Gaz:    %4d",  S.gasADC);
      break;
    case 2:
      snprintf(l1,17,"Alarma:%-9s",  S.alarmArmed ? "ARMATA" : "Inactiva");
      snprintf(l2,17,"Card:  %-9s",  S.cardIdx >= 0 ? "Activ" : "Niciun");
      break;
    case 3:
      snprintf(l1,17,"PIR: %-11s",   S.pir ? "MISCARE!" : "Liniste");
      snprintf(l2,17,"%-16s",        WiFi.isConnected()
               ? WiFi.localIP().toString().c_str() : "WiFi offline");
      break;
  }
  lcdP(String(l1), String(l2));
  S.lcdPage = (S.lcdPage + 1) % 4;
}

// ============================================================
//  RFID
// ============================================================
void handleRFID() {

  if (S.waitCard && millis() > S.waitCardExp) {
    S.waitCard = false;
    addLog("Timeout card nou");
    lcdOvr("Timeout!", "Anulat.", 2000);
  }

  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  uint8_t* uid = rfid.uid.uidByte;
  String   us  = uidS(uid, CARD_UID_LEN);
  Serial.printf("[RFID] Card: %s\n", us.c_str());

  if (S.waitCard) {
    S.waitCard = false;
    if (findCard(uid) >= 0) {
      lcdOvr("Exista deja!", us, 2000);
    } else if (cardCount < MAX_RFID_CARDS) {
      RFIDCard& c = cards[cardCount];
      memset(&c, 0, sizeof(RFIDCard));
      memcpy(c.uid, uid, CARD_UID_LEN);
      c.valid=true; c.ledR=255; c.ledG=255; c.ledB=255;
      c.ledOnDay=false; c.ledOnNight=true; c.buzzerOnRead=true;
      cardCount++;
      saveNVM();
      addLog(("Card nou: " + us).c_str());

      StaticJsonDocument<128> d;
      d["event"]="new_card"; d["uid"]=us; d["index"]=cardCount-1;
      char b[128]; serializeJson(d,b); mqtt.publish(TOPIC_RFID_NEW, b);

      lcdOvr("Card adaugat!", us, 3000);
      buzBeep(2, 80, 80);
    } else {
      lcdOvr("Memorie plina!", "Max 20 carduri", 2000);
    }
    rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
    return;
  }

  // === VALIDARE ACCES ===
  int idx = findCard(uid);
  if (idx >= 0) {
    S.cardIdx = idx;
    RFIDCard& c = cards[idx];
    bool wasArmed = S.alarmArmed;

    // Dezarmare
    S.alarmArmed = S.alarmMovSilent = S.alarmMovActive = false;
    if (wasArmed) buzStop();

    if (c.buzzerOnRead) buzBeep(1, 120, 0);

    S.ledManual = false;
    applyCardLED(idx);

    if (wasArmed) {
      lcdOvr("Alarma Dezarmata", "Access: " + us.substring(0,8), 3000);
      addLog(("Dezarmat: " + us).c_str());
    } else {
      lcdOvr("Access OK", us, 2000);
      addLog(("Access OK: " + us).c_str());
    }
    StaticJsonDocument<128> d; d["event"]="access_granted"; d["uid"]=us;
    char b[128]; serializeJson(d,b); mqtt.publish(TOPIC_STATUS, b);

  } else {
    buzBeep(3, 180, 120);
    lcdOvr("Access REFUZAT!", us, 3000);
    addLog(("Refuzat: " + us).c_str());
    StaticJsonDocument<128> d; d["event"]="access_denied"; d["uid"]=us;
    char b[128]; serializeJson(d,b); mqtt.publish(TOPIC_STATUS, b);
  }

  rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
}

// ============================================================
//  BUTTON
// ============================================================
void handleButton() {
  static bool          last  = HIGH;
  static unsigned long pressT = 0;
  bool cur = digitalRead(PIN_BUTTON);
  if (cur == LOW  && last == HIGH) pressT = millis();
  if (cur == HIGH && last == LOW) {
    unsigned long d = millis() - pressT;
    if (d < 30) { /* bounce */ }
    else if (d < 1200) {
      S.alarmArmed = !S.alarmArmed;
      if (!S.alarmArmed) { S.alarmMovSilent=S.alarmMovActive=false; buzStop(); }
      addLog(S.alarmArmed ? "Armat (buton)" : "Dezarmat (buton)");
      lcdOvr(S.alarmArmed ? "Alarma: ARMATA" : "Alarma: Dezarm.", "Buton apasat", 2000);
      buzBeep(S.alarmArmed ? 2 : 1, 100, 80);
    } else {
      S.ledManual = false; S.cardIdx = -1;
      addLog("Reset LED (buton lung)");
      lcdOvr("LED resetat", "Mod implicit", 2000);
    }
  }
  last = cur;
}

// ============================================================
//  NVM
// ============================================================
void saveNVM() {
  prefs.putInt("cnt", cardCount);
  for (int i = 0; i < cardCount; i++) {
    char k[10]; snprintf(k, 10, "c%d", i);
    prefs.putBytes(k, &cards[i], sizeof(RFIDCard));
  }
  Serial.printf("[NVM] Salvate %d carduri\n", cardCount);
}

void loadNVM() {
  cardCount = prefs.getInt("cnt", 0);
  if (cardCount < 0 || cardCount > MAX_RFID_CARDS) cardCount = 0;
  for (int i = 0; i < cardCount; i++) {
    char k[10]; snprintf(k, 10, "c%d", i);
    if (prefs.getBytes(k, &cards[i], sizeof(RFIDCard)) != sizeof(RFIDCard)) {
      cardCount = i; break;
    }
  }
  Serial.printf("[NVM] Incarcate %d carduri\n", cardCount);
}

// ============================================================
//  LOG
// ============================================================
void addLog(const char* msg) {
  logBuf[logHead].ts = millis();
  strncpy(logBuf[logHead].msg, msg, LOG_L-1);
  logBuf[logHead].msg[LOG_L-1] = '\0';
  logHead = (logHead+1) % LOG_N;
  if (logCnt < LOG_N) logCnt++;
  Serial.printf("[LOG] %s\n", msg);
}

void sendLog() {
  int start = (logHead - logCnt + LOG_N) % LOG_N;
  for (int sent = 0; sent < logCnt; ) {
    StaticJsonDocument<768> doc;
    JsonArray arr = doc.createNestedArray("log");
    doc["total"] = logCnt; doc["from"] = sent;
    int batch = min(logCnt - sent, 5);
    for (int i = 0; i < batch; i++) {
      int x = (start + sent + i) % LOG_N;
      JsonObject o = arr.createNestedObject();
      o["ts"]=logBuf[x].ts; o["msg"]=logBuf[x].msg;
    }
    char b[768]; serializeJson(doc,b);
    mqtt.publish(TOPIC_LOG, b);
    sent += batch; delay(30);
  }
  addLog("Log trimis MQTT");
}

// ============================================================
//  UTILITARE
// ============================================================
String uidS(uint8_t* uid, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += '0';
    s += String(uid[i], HEX);
    if (i < len-1) s += ':';
  }
  s.toUpperCase(); return s;
}

bool uidEq(uint8_t* a, uint8_t* b) {
  for (int i = 0; i < CARD_UID_LEN; i++) if (a[i]!=b[i]) return false;
  return true;
}

int findCard(uint8_t* uid) {
  for (int i = 0; i < cardCount; i++)
    if (cards[i].valid && uidEq(cards[i].uid, uid)) return i;
  return -1;
}
