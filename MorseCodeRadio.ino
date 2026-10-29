#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>
#include "driver/ledc.h"

#define I2C_SDA_PIN       21
#define I2C_SCL_PIN       22
#define MORSE_BTN_PIN     18
#define CHAR_BTN_PIN      19
#define BUZZER_PIN        25

#define SCREEN_WIDTH      128
#define SCREEN_HEIGHT     64
#define OLED_RESET        -1
#define OLED_ADDR         0x3C

const uint32_t DASH_THRESHOLD_MS   = 250;   
const uint32_t DEBOUNCE_MS         = 30;
const uint32_t CLEAR_HOLD_MS       = 1500;
const uint32_t CHAR_GAP_MS         = 400;   
const uint32_t PLAYBACK_UNIT_MS    = 80;
const uint32_t PLAYBACK_DASH_MS    = 240;
const uint32_t TONE_FREQ           = 750;
const uint32_t DISPLAY_REFRESH_MS  = 100;
const uint32_t RX_FLASH_MS         = 3000;

const uint32_t SPLASH_DURATION_MS  = 3000;

const int ESPNOW_CHANNEL = 1;
const int MSG_BUF_SIZE   = 96;
const int MAX_MORSE_LEN  = 12;

#define USE_UNICAST 0
#if USE_UNICAST
const uint8_t PEER_MAC[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
#endif

const uint8_t PKT_MAGIC      = 0x5A;
const uint8_t PKT_TYPE_CHAR  = 1;
const uint8_t PKT_TYPE_SPACE = 2;

struct __attribute__((packed)) MorsePacket {
  uint8_t magic;
  uint8_t type;
  char    ch;
};

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

QueueHandle_t buzzerQueue = nullptr;

portMUX_TYPE txMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

char txBuf[MSG_BUF_SIZE] = {0};
int  txLen = 0;
char rxBuf[MSG_BUF_SIZE] = {0};
int  rxLen = 0;

String morsePattern = "";
String decodeStatus = "";
const char* wifiStatus = "INIT";
bool lrEnabled = false;
int8_t txPowerQuarterDbm = 0;   

volatile bool displayDirty = true;
uint32_t lastDisplayUpdate = 0;

uint8_t destAddr[6];

uint32_t rxFlashStart  = 0;
char     rxFlashChar   = 0;
bool     rxFlashActive = false;

uint32_t lastMorseReleaseTime = 0;

struct DebouncedButton {
  uint8_t  pin;
  bool     stable;
  bool     lastRaw;
  uint32_t lastDebounce;
  uint32_t pressStart;
};

DebouncedButton morseBtn = {MORSE_BTN_PIN, false, false, 0, 0};
DebouncedButton charBtn  = {CHAR_BTN_PIN,  false, false, 0, 0};

uint32_t bothPressStart = 0;
bool clearTriggered = false;
bool suppressIndividual = false;

bool     buzzerActive = false;
String   buzzerPattern = "";
int      buzzerIndex = 0;
bool     buzzerToneOn = false;
uint32_t buzzerPhaseStart = 0;
uint32_t buzzerPhaseDuration = 0;

struct MorseEntry { char ch; const char* code; };
const MorseEntry MORSE_TABLE[] = {
  {'A', ".-"},    {'B', "-..."},  {'C', "-.-."},  {'D', "-.."},
  {'E', "."},     {'F', "..-."},  {'G', "--."},   {'H', "...."},
  {'I', ".."},    {'J', ".---"},  {'K', "-.-"},   {'L', ".-.."},
  {'M', "--"},    {'N', "-."},    {'O', "---"},   {'P', ".--."},
  {'Q', "--.-"},  {'R', ".-."},   {'S', "..."},   {'T', "-"},
  {'U', "..-"},   {'V', "...-"},  {'W', ".--"},   {'X', "-..-"},
  {'Y', "-.--"},  {'Z', "--.."},
  {'0', "-----"}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
  {'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."},
  {'8', "---.."}, {'9', "----."},
  {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."}, {'/', "-..-."},
  {'@', ".--.-."}, {'-', "-....-"}, {'(', "-.--."},  {')', "-.--.-"},
  {'"', ".-..-."}, {'\'', ".----."}, {':', "---..."}, {';', "-.-.-."},
  {'=', "-...-"},  {'+', ".-.-."},  {'_', "..--.-"}, {'$', "...-..-"},
  {'!', "-.-.--"}
};
const int MORSE_TABLE_LEN = sizeof(MORSE_TABLE) / sizeof(MORSE_TABLE[0]);

#define BUZZER_LEDC_TIMER    LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL  LEDC_CHANNEL_0
#define BUZZER_LEDC_MODE     LEDC_HIGH_SPEED_MODE
#define BUZZER_LEDC_RES      LEDC_TIMER_10_BIT
#define BUZZER_LEDC_DUTY     512

void buzzerInit() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode      = BUZZER_LEDC_MODE;
  tcfg.duty_resolution = BUZZER_LEDC_RES;
  tcfg.timer_num       = BUZZER_LEDC_TIMER;
  tcfg.freq_hz         = TONE_FREQ;
  ledc_timer_config(&tcfg);

  ledc_channel_config_t ccfg = {};
  ccfg.gpio_num   = BUZZER_PIN;
  ccfg.speed_mode = BUZZER_LEDC_MODE;
  ccfg.channel    = BUZZER_LEDC_CHANNEL;
  ccfg.intr_type  = LEDC_INTR_DISABLE;
  ccfg.timer_sel  = BUZZER_LEDC_TIMER;
  ccfg.duty       = 0;
  ccfg.hpoint     = 0;
  ledc_channel_config(&ccfg);
}

void pcdTone(uint32_t freq) {
  ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq);
  ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_LEDC_DUTY);
  ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void pcdNoTone() {
  ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
  ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
}

void initDisplay();
void showSplashScreen();          
void initESPNow();
void renderDisplay();
void renderNormalView(const char* txCopy, const char* rxCopy);
void renderReceiveView(const char* rxCopy);
void updateButtons();
void updateButton(DebouncedButton &btn, bool raw, bool isMorse);
void handleMorseRelease(uint32_t duration);
void handleCharRelease(uint32_t duration);
void checkAutoCommit();
void commitPattern();
void addSpace();
void clearMessages();
void appendToBuffer(char* buf, int &len, char c, portMUX_TYPE* mux);
void sendPacket(uint8_t type, char ch);
void handleReceivedPacket(const uint8_t *data, int len);
void updateBuzzer();
void startBuzzer(char c);
const char* morseForChar(char c);
char morseToChar(const String &pattern);
String previewMorse(const String &p);
bool trySegment(const String &p, String &result);
String truncateLast(const char* buf, int maxLen);
String visibleSpaces(const String &in);
void printProtocolBitmap(uint8_t proto);

#if ESP_IDF_VERSION_MAJOR >= 5
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send failed");
}
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  (void)info;
  handleReceivedPacket(data, len);
}
#else
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  if (status != ESP_NOW_SEND_SUCCESS) Serial.println("ESP-NOW send failed");
}
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
  (void)mac_addr;
  handleReceivedPacket(data, len);
}
#endif

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("PCD starting (real-morse, no auto-space, LR)");

  pinMode(MORSE_BTN_PIN, INPUT_PULLUP);
  pinMode(CHAR_BTN_PIN,  INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  buzzerInit();
  buzzerQueue = xQueueCreate(16, sizeof(char));

  initDisplay();
  showSplashScreen();     
  initESPNow();

  displayDirty = true;
  renderDisplay();
}

void loop() {
  updateButtons();

  if (rxFlashActive && (morseBtn.stable || charBtn.stable)) {
    rxFlashActive = false;
    displayDirty = true;
  }

  checkAutoCommit();
  updateBuzzer();

  if (rxFlashActive && (millis() - rxFlashStart >= RX_FLASH_MS)) {
    rxFlashActive = false;
    displayDirty = true;
  }

  if (displayDirty && (millis() - lastDisplayUpdate >= DISPLAY_REFRESH_MS)) {
    renderDisplay();
  }

  delay(1);
}

void checkAutoCommit() {
  if (morseBtn.stable) return;
  if (morsePattern.length() == 0) return;
  if (millis() - lastMorseReleaseTime < CHAR_GAP_MS) return;

  commitPattern();
}

void initDisplay() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 init failed. Check address and wiring.");
    return;
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("PCD INIT");
  display.display();
}

void showSplashScreen() {
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);

  const char* text = "A.S.A";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH  - (int16_t)w) / 2 - x1;
  int16_t y = (SCREEN_HEIGHT - (int16_t)h) / 2 - y1;

  display.setCursor(x, y);
  display.print(text);
  display.display();

  Serial.println("Splash: A.S.A (3 s)");
  delay(SPLASH_DURATION_MS);
}

void renderDisplay() {
  char txCopy[MSG_BUF_SIZE], rxCopy[MSG_BUF_SIZE];

  portENTER_CRITICAL(&txMux);
  strncpy(txCopy, txBuf, MSG_BUF_SIZE);
  txCopy[MSG_BUF_SIZE - 1] = '\0';
  portEXIT_CRITICAL(&txMux);

  portENTER_CRITICAL(&rxMux);
  strncpy(rxCopy, rxBuf, MSG_BUF_SIZE);
  rxCopy[MSG_BUF_SIZE - 1] = '\0';
  portEXIT_CRITICAL(&rxMux);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  if (rxFlashActive) renderReceiveView(rxCopy);
  else               renderNormalView(txCopy, rxCopy);

  display.display();
  displayDirty = false;
  lastDisplayUpdate = millis();
}

void renderNormalView(const char* txCopy, const char* rxCopy) {
  display.setCursor(0, 0);
  display.print("PCD ");
  display.print(wifiStatus);
  display.print(lrEnabled ? " LR" : " noLR");

  display.setCursor(0, 10);
  display.print("M:");
  if (morsePattern.length() > 0) {
    display.print(morsePattern);
    display.print(" (");
    display.print(previewMorse(morsePattern));
    display.print(")");
  } else {
    display.print("-");
    if (decodeStatus.length() > 0) {
      display.print(" ");
      display.print(decodeStatus);
    }
  }

  display.setCursor(0, 20);
  display.print("TX:");
  display.print(visibleSpaces(truncateLast(txCopy, 18)));

  display.setCursor(0, 30);
  display.print("RX:");
  display.print(visibleSpaces(truncateLast(rxCopy, 18)));

  display.setCursor(0, 44);
  display.print("tap=dot  hold=dash");

  display.setCursor(0, 54);
  display.print("char=space  both=CLR");
}

void renderReceiveView(const char* rxCopy) {
  display.setCursor(0, 0);
  display.print("PCD ");
  display.print(wifiStatus);
  display.print(lrEnabled ? " LR" : " noLR");
  display.print(" <<RX");

  display.setCursor(0, 10);
  display.print("Morse: ");
  if (rxFlashChar == ' ') {
    display.print("(space)");
  } else {
    const char* mp = morseForChar(rxFlashChar);
    if (mp) { display.print(mp); display.print(" = "); display.print(rxFlashChar); }
    else    { display.print("?"); }
  }

  display.setCursor(0, 22);
  display.print("Received:");

  String rxView = visibleSpaces(truncateLast(rxCopy, 63));
  int y = 34;
  for (int i = 0; i < (int)rxView.length() && y <= 54; i += 21) {
    display.setCursor(0, y);
    int end = min((int)rxView.length(), i + 21);
    for (int j = i; j < end; j++) display.print(rxView[j]);
    y += 10;
  }
}

String truncateLast(const char* buf, int maxLen) {
  int len = strlen(buf);
  if (len <= maxLen) return String(buf);
  String out = "..";
  out += String(buf + len - (maxLen - 2));
  return out;
}

String visibleSpaces(const String &in) {
  String out = in;
  out.replace(' ', '_');
  return out;
}

String previewMorse(const String &p) {
  if (p.length() == 0) return "";
  char single = morseToChar(p);
  if (single != '\0') return String(single);
  String seg = "";
  if (trySegment(p, seg)) return seg;
  return "?";
}

void printProtocolBitmap(uint8_t proto) {
  Serial.print("  bits: ");
  if (proto & WIFI_PROTOCOL_11B)  Serial.print("11B ");
  if (proto & WIFI_PROTOCOL_11G)  Serial.print("11G ");
  if (proto & WIFI_PROTOCOL_11N)  Serial.print("11N ");
  if (proto & WIFI_PROTOCOL_LR)   Serial.print("LR ");
#ifdef WIFI_PROTOCOL_11AX
  if (proto & WIFI_PROTOCOL_11AX) Serial.print("11AX ");
#endif
  Serial.println();
}

void initESPNow() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  esp_wifi_set_ps(WIFI_PS_NONE);

#ifdef WIFI_PROTOCOL_LR
  esp_err_t err = esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  if (err != ESP_OK) {
    Serial.print("esp_wifi_set_protocol(LR) failed: 0x");
    Serial.println(err, HEX);
  }
#else
  Serial.println("WIFI_PROTOCOL_LR not defined");
#endif

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(100);

  esp_wifi_set_max_tx_power(84);
  esp_wifi_get_max_tx_power(&txPowerQuarterDbm);

  uint8_t proto = 0;
  esp_wifi_get_protocol(WIFI_IF_STA, &proto);
  lrEnabled = (proto & WIFI_PROTOCOL_LR) != 0;

  Serial.print("MAC: ");            Serial.println(WiFi.macAddress());
  uint8_t primary = 0; wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
  esp_wifi_get_channel(&primary, &second);
  Serial.print("Wi-Fi channel: ");  Serial.println(primary);
  Serial.print("Protocol bitmap: 0x"); Serial.println(proto, HEX);
  printProtocolBitmap(proto);
  Serial.print("Long Range (verified): "); Serial.println(lrEnabled ? "YES" : "NO");
  Serial.print("TX power: ");
  Serial.print(txPowerQuarterDbm / 4.0, 2);
  Serial.println(" dBm");

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    wifiStatus = "ERROR";
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memset(destAddr, 0xFF, 6);
#if USE_UNICAST
  memcpy(destAddr, PEER_MAC, 6);
#endif

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, destAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: failed to add ESP-NOW peer");
    wifiStatus = "ERROR";
    return;
  }

#if USE_UNICAST
  Serial.println("Mode: UNICAST (hardware ACK + retry active)");
#else
  Serial.println("Mode: BROADCAST (no hardware ACK/retry — see header comment)");
#endif

  wifiStatus = "READY";
  Serial.println("ESP-NOW ready");
}

void sendPacket(uint8_t type, char ch) {
  MorsePacket pkt;
  pkt.magic = PKT_MAGIC;
  pkt.type  = type;
  pkt.ch    = ch;
  esp_err_t r = esp_now_send(destAddr, (uint8_t*)&pkt, sizeof(pkt));
  if (r != ESP_OK) {
    Serial.print("esp_now_send error: 0x");
    Serial.println(r, HEX);
  }
}

void handleReceivedPacket(const uint8_t *data, int len) {
  if (len != (int)sizeof(MorsePacket)) { Serial.println("RX bad len"); return; }
  MorsePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != PKT_MAGIC) { Serial.println("RX bad magic"); return; }

  if (pkt.type == PKT_TYPE_CHAR) {
    appendToBuffer(rxBuf, rxLen, pkt.ch, &rxMux);
    xQueueSend(buzzerQueue, &pkt.ch, 0);
    rxFlashChar = pkt.ch;
    rxFlashStart = millis();
    rxFlashActive = true;
    displayDirty = true;
    Serial.print("RX: "); Serial.println(pkt.ch);
  } else if (pkt.type == PKT_TYPE_SPACE) {
    char sp = ' ';
    appendToBuffer(rxBuf, rxLen, sp, &rxMux);
    xQueueSend(buzzerQueue, &sp, 0);
    rxFlashChar = ' ';
    rxFlashStart = millis();
    rxFlashActive = true;
    displayDirty = true;
    Serial.println("RX: space");
  }
}

void appendToBuffer(char* buf, int &len, char c, portMUX_TYPE* mux) {
  portENTER_CRITICAL(mux);
  if (len < MSG_BUF_SIZE - 1) {
    buf[len++] = c;
    buf[len] = '\0';
  } else {
    memmove(buf, buf + 1, MSG_BUF_SIZE - 2);
    buf[MSG_BUF_SIZE - 2] = c;
    buf[MSG_BUF_SIZE - 1] = '\0';
    len = MSG_BUF_SIZE - 1;
  }
  portEXIT_CRITICAL(mux);
}

void updateButtons() {
  bool rawMorse = (digitalRead(MORSE_BTN_PIN) == LOW);
  bool rawChar  = (digitalRead(CHAR_BTN_PIN)  == LOW);

  updateButton(morseBtn, rawMorse, true);
  updateButton(charBtn,  rawChar,  false);

  if (morseBtn.stable && charBtn.stable) {
    if (bothPressStart == 0) bothPressStart = millis();
    if (!clearTriggered && (millis() - bothPressStart >= CLEAR_HOLD_MS)) {
      clearMessages();
      clearTriggered = true;
      suppressIndividual = true;
    }
  } else {
    bothPressStart = 0;
    if (clearTriggered && !morseBtn.stable && !charBtn.stable) {
      clearTriggered = false;
      suppressIndividual = false;
    }
  }
}

void updateButton(DebouncedButton &btn, bool raw, bool isMorse) {
  if (raw != btn.lastRaw) { btn.lastRaw = raw; btn.lastDebounce = millis(); }
  if ((millis() - btn.lastDebounce) > DEBOUNCE_MS && raw != btn.stable) {
    btn.stable = raw;
    if (btn.stable) {
      btn.pressStart = millis();
    } else {
      uint32_t d = millis() - btn.pressStart;
      if (!suppressIndividual) {
        if (isMorse) handleMorseRelease(d);
        else         handleCharRelease(d);
      }
    }
  }
}

void handleMorseRelease(uint32_t duration) {
  morsePattern += (duration >= DASH_THRESHOLD_MS) ? '-' : '.';
  lastMorseReleaseTime = millis();

  if ((int)morsePattern.length() > MAX_MORSE_LEN) {
    morsePattern = "";
    decodeStatus = "BAD";
  } else {
    decodeStatus = "";
  }
  displayDirty = true;
}

void handleCharRelease(uint32_t duration) {
  (void)duration;

  if (morsePattern.length() > 0) {
    commitPattern();
  }

  addSpace();

  decodeStatus = "SPC";
  displayDirty = true;
}

void commitPattern() {
  if (morsePattern.length() == 0) return;

  char single = morseToChar(morsePattern);
  if (single != '\0') {
    appendToBuffer(txBuf, txLen, single, &txMux);
    sendPacket(PKT_TYPE_CHAR, single);
    decodeStatus = "OK";
    Serial.print("Commit: "); Serial.println(single);
    morsePattern = "";
    displayDirty = true;
    return;
  }

  String seg = "";
  if (trySegment(morsePattern, seg)) {
    Serial.print("Segment: "); Serial.println(seg);
    for (int i = 0; i < (int)seg.length(); i++) {
      appendToBuffer(txBuf, txLen, seg[i], &txMux);
      sendPacket(PKT_TYPE_CHAR, seg[i]);
    }
    decodeStatus = "OK";
  } else {
    Serial.print("Invalid: "); Serial.println(morsePattern);
    decodeStatus = "BAD";
  }
  morsePattern = "";
  displayDirty = true;
}

void addSpace() {
  appendToBuffer(txBuf, txLen, ' ', &txMux);
  sendPacket(PKT_TYPE_SPACE, ' ');
  Serial.println("Space");
}

void clearMessages() {
  portENTER_CRITICAL(&txMux);
  txLen = 0; txBuf[0] = '\0';
  portEXIT_CRITICAL(&txMux);

  portENTER_CRITICAL(&rxMux);
  rxLen = 0; rxBuf[0] = '\0';
  portEXIT_CRITICAL(&rxMux);

  morsePattern = "";
  decodeStatus = "CLR";
  rxFlashActive = false;
  rxFlashChar = 0;

  pcdNoTone();
  buzzerActive = false;
  buzzerPattern = "";
  buzzerIndex = 0;
  buzzerToneOn = false;
  if (buzzerQueue) xQueueReset(buzzerQueue);

  displayDirty = true;
  Serial.println("Cleared");
}

char morseToChar(const String &p) {
  for (int i = 0; i < MORSE_TABLE_LEN; i++)
    if (p == MORSE_TABLE[i].code) return MORSE_TABLE[i].ch;
  return '\0';
}
const char* morseForChar(char c) {
  for (int i = 0; i < MORSE_TABLE_LEN; i++)
    if (MORSE_TABLE[i].ch == c) return MORSE_TABLE[i].code;
  return nullptr;
}

bool trySegmentRec(const String &p, String &result, int &steps) {
  if (++steps > 200) return false;
  if (p.length() == 0) return true;
  if (result.length() > 12) return false;
  
  const int order[] = {3, 2, 4, 5, 6, 7, 1};
  for (int i = 0; i < 7; i++) {
    int len = order[i];
    if (len > (int)p.length()) continue;
    char c = morseToChar(p.substring(0, len));
    if (c != '\0') {
      result += c;
      if (trySegmentRec(p.substring(len), result, steps)) return true;
      result.remove(result.length() - 1);
    }
  }
  return false;
}
bool trySegment(const String &p, String &result) {
  result = "";
  int steps = 0;
  return trySegmentRec(p, result, steps);
}
void updateBuzzer() {
  if (!buzzerActive) {
    char c;
    if (xQueueReceive(buzzerQueue, &c, 0) == pdTRUE) startBuzzer(c);
    return;
  }
  uint32_t now = millis();
  if (now - buzzerPhaseStart < buzzerPhaseDuration) return;

  if (buzzerToneOn) {
    pcdNoTone();
    buzzerToneOn = false;
    buzzerPhaseDuration =
        (buzzerIndex >= (int)buzzerPattern.length()) ? 3 * PLAYBACK_UNIT_MS
                                                     : PLAYBACK_UNIT_MS;
    buzzerPhaseStart = now;
  } else {
    if (buzzerIndex < (int)buzzerPattern.length()) {
      char e = buzzerPattern[buzzerIndex++];
      buzzerPhaseDuration = (e == '.') ? PLAYBACK_UNIT_MS : PLAYBACK_DASH_MS;
      pcdTone(TONE_FREQ);
      buzzerToneOn = true;
      buzzerPhaseStart = now;
    } else {
      buzzerActive = false;
    }
  }
}
void startBuzzer(char c) {
  if (c == ' ') {
    buzzerPattern = "";
    buzzerIndex = 0;
    buzzerToneOn = false;
    buzzerPhaseDuration = 7 * PLAYBACK_UNIT_MS;
    buzzerPhaseStart = millis();
    buzzerActive = true;
    return;
  }
  const char* p = morseForChar(c);
  if (!p || p[0] == '\0') return;
  buzzerPattern = String(p);
  buzzerIndex = 0;
  buzzerToneOn = false;
  buzzerPhaseDuration = 0;
  buzzerPhaseStart = millis() - 1;
  buzzerActive = true;
}