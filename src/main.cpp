// --------------------------------------------------------
//  *** HC-SR04-Cardputer ***     by NoRi
//  HC-SR04 Ultrasonic Senseor software for Cardputer
// ( distance measurement )
//    2025-06-30  v101
// https://github.com/NoRi-230401/HC-SR04-Cardputer
//  MIT License
// --------------------------------------------------------
#include "N_util.h"

static unsigned long SR04_CHECK_INTERVAL_MS = 1 * 1000UL;

enum KeyNum
{
  KN_NONE,
  KN_UP,
  KN_DOWN,
  KN_LEFT,
  KN_RIGHT
};

enum SettingMode
{
  SM_ESC,
  SM_BRIGHT_LEVEL,
  SM_LOWBAT_THRESHOLD,
  SM_LANG
};
static SettingMode settingMode = SM_ESC;

#define BRIGHT_LVL_INIT 30
#define BRIGHT_LVL_MAX 255
#define BRIGHT_LVL_MIN 0
#define BATLVL_MAX 100
#define LOWBAT_THRESHOLD_INIT 10
#define LOWBAT_THRESHOLD_MAX 95
#define LOWBAT_THRESHOLD_MIN 5
#define LANG_INIT 0 // 0:English 1:Japanese
#define LANG_MAX 1
#define BATLVL_ITEM_POS 22 // 'bat.' or '電池'
#define BATLVL_ITEM_LEN 4
#define BATLVL_VALUE_POS 26 // xxx
#define BATLVL_VALUE_LEN 3
#define BATLVL_PERCENT_POS 29 // %
#define SETTING_DISP_POS 2    // setting display start position
#define MEAS_UNIT_POS 23      // cm
#define MEAS_ITEM_POS 2

// --- Key mapping constants ---
const char KEY_SETTING_ESCAPE = '`';
const char KEY_SETTING_BRIGHTNESS = '1';
const char KEY_SETTING_LOWBAT = '2';
const char KEY_SETTING_LANG = '3';
const char KEY_UP = ';';
const char KEY_DOWN = '.';
const char KEY_LEFT = ',';
const char KEY_RIGHT = '/';

const char *BATLVL_TITLE[] = {"bat.", "電池"};
static uint8_t BRIGHT_LVL;       // 0 - 255 : LCD bright level
static uint8_t LOWBAT_THRESHOLD; // 5 - 95% : LOW BATTERY Threshold level
const char *NVM_BRIGHT = "brt";
const char *NVM_LOWBAT = "lbat";
const char *NVM_LANG = "lang";
const char *LANG[] = {"English", "日本語"};
static uint8_t LANG_INDEX = 0;
const char *meas_items[] = {"Distance", "距離"};

void setup();
void loop();
void SR04_sensor();
void dispInit();
bool keyCheck();
void settings();
void changeSettings(SettingMode mode, KeyNum keyNo);
void changeLang(KeyNum keyNo);
bool updateLang(KeyNum keyNo);
void dispBatItem();
void dispMeasItem();
bool updateSettingValue(uint8_t &value, KeyNum keyNo, uint8_t min, uint8_t max, uint8_t step, uint8_t big_step);
void prtSetting(const char *msg, uint8_t data);
void prtSetting(const char *msg, const char *data);
void changeBright(KeyNum keyNo);
void changeLowBatThr(KeyNum keyNo);
void settingsInit();
void prtDistance(double temp_val);
void batteryState();
void prtBatLvl(uint8_t batLvl);
void lowBatteryCheck(uint8_t batLvl);

// --------------------------------------------------------
// --- For non-blocking HC-SR04 reading ---
volatile unsigned long echoStartTime = 0;
volatile unsigned long echoEndTime = 0;
volatile bool echoReceived = false;
void IRAM_ATTR echo_isr();

#define echoPin 1 // Echo Pin
#define trigPin 2 // Trigger Pin

void setup()
{
  m5stack_begin();

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);

  // Attach interrupt to the echo pin
  attachInterrupt(digitalPinToInterrupt(echoPin), echo_isr, CHANGE);

  if (SD_ENABLE)
  { // M5stack-SD-Updater lobby
    SDU_lobby();
    SD.end();
  }

  settingsInit();
  dispInit();
  canvas.pushSprite(0, 0);
}

void loop()
{
  SR04_sensor();
  batteryState();

  if (keyCheck())
    settings();

  vTaskDelay(1);
}

#define DIST_LINE_INDEX 3
#define DIST_DISP_WIDTH 27
static unsigned long prev_sr04_trigger_ms = 0L;
static bool sr04_triggered = false; // Flag to indicate a trigger pulse was sent

void SR04_sensor()
{
  unsigned long current_ms = millis();

  // Trigger the sensor at regular intervals if not waiting for an echo
  if (!sr04_triggered && (current_ms - prev_sr04_trigger_ms >= SR04_CHECK_INTERVAL_MS))
  {
    prev_sr04_trigger_ms = current_ms;
    echoReceived = false;
    sr04_triggered = true; // Set flag that we are waiting for an echo

    // Send trigger pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  }

  // Check if a new echo has been received
  if (echoReceived)
  {
    sr04_triggered = false; // Reset trigger flag
    // Disable interrupts temporarily to safely read volatile variables
    noInterrupts();
    unsigned long duration = echoEndTime - echoStartTime;
    echoReceived = false; // Reset the flag
    interrupts();

    // Check for valid duration (e.g., less than 38ms for ~6.5m range)
    if (duration > 0 && duration < 38000) {
        // Speed of sound in cm/us (at approx. 20°C)
        const double soundVelocity = 34350.0 / 1000000.0;
        double distance = duration * soundVelocity / 2; // [cm]
        prtDistance(distance);
        dbPrtln("Distance = " + String(distance) + " cm");
    } else {
        // Duration too long or zero, likely an error or out of range
        prtDistance(NAN);
        dbPrtln("Distance = NAN");
    }
    canvas.pushSprite(0, 0);
  }
  // Check for timeout (e.g., 60ms is a reasonable timeout for HC-SR04)
  else if (sr04_triggered && (current_ms - prev_sr04_trigger_ms > 60))
  {
    sr04_triggered = false; // Reset trigger flag
    prtDistance(NAN);       // Report timeout as Not-A-Number
    canvas.pushSprite(0, 0);
  }
}

void IRAM_ATTR echo_isr() {
  if (digitalRead(echoPin) == HIGH) {
    echoStartTime = micros();
  } else {
    echoEndTime = micros();
    echoReceived = true;
  }
}

static float PREV_DISTANCE = 0.0;
void prtDistance(double temp_val)
{
  // Note: (NAN == NAN) is always false, so we need isnan() for the check.
  if (isnan(PREV_DISTANCE) && isnan(temp_val))
  {
    return;
  }
  PREV_DISTANCE = temp_val;

  char buf[10];
  if (isnan(temp_val))
  {
    snprintf(buf, sizeof(buf), "---.-");
  }
  else
  {
    snprintf(buf, sizeof(buf), "%3.1f", temp_val);
  }

  const int font_size = 48;
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setFont(&fonts::Font7);
  canvas.setTextSize(1);
  canvas.fillRect(0, SC_LINES[DIST_LINE_INDEX], X_WIDTH, font_size, TFT_BLACK);
  canvas.drawCenterString(buf, X_WIDTH / 2, SC_LINES[DIST_LINE_INDEX]);
}

void dispInit()
{
  // ---012345678901234567890123456789----
  // L0:- HC-SR04 Sensor -    bat.---%
  // L1: (settings display line)
  // L2:
  // L3:
  // L4:
  // L5:
  // L6:
  // L7:  Distance                cm
  // ---012345678901234567890123456789----

  canvas.fillScreen(TFT_BLACK); // all clear
  // canvas.setFont(&fonts::lgfxJapanMincho_16);
  canvas.setFont(&fonts::lgfxJapanGothic_16);

  //--L0 : title--------------
  canvas.setTextColor(TFT_SKYBLUE, TFT_BLACK);
  canvas.drawString(F("- HC-SR04 Sensor -"), 0, SC_LINES[0]);

  // L0 :Battery Level -----
  dispBatItem();
  canvas.drawString(F("---"), W_CHR * BATLVL_VALUE_POS, SC_LINES[0]);
  canvas.drawString(F("%"), W_CHR * BATLVL_PERCENT_POS, SC_LINES[0]);

  // L7 : Measuremnt items
  canvas.setTextColor(TFT_GREEN, TFT_BLACK);
  canvas.drawString(F("cm"), W_CHR * MEAS_UNIT_POS, SC_LINES[7], &fonts::Font4);
  dispMeasItem();
}

bool keyCheck()
{
  M5Cardputer.update(); // update Cardputer key input

  if (M5Cardputer.Keyboard.isChange())
  {
    if (M5Cardputer.Keyboard.isPressed())
      return true;
  }
  return false;
}

void settings()
{
  // Setting Mode Check
  KeyNum keyNum = KN_NONE;

  // key 1 - 3 : special setting mode
  // key `     : clear and escape special mode
  if (M5Cardputer.Keyboard.isKeyPressed(KEY_SETTING_ESCAPE)) // clear and escape
  {
    if (settingMode == SM_ESC)
      return;
    settingMode = SM_ESC;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_SETTING_BRIGHTNESS)) // lcd brightness
  {
    if (settingMode == SM_BRIGHT_LEVEL)
      return;
    settingMode = SM_BRIGHT_LEVEL;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_SETTING_LOWBAT)) // lowBattery threshold
  {
    if (settingMode == SM_LOWBAT_THRESHOLD)
      return;
    settingMode = SM_LOWBAT_THRESHOLD;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_SETTING_LANG)) // select language
  {
    if (settingMode == SM_LANG)
      return;
    settingMode = SM_LANG;
  }
  // -----------------------------------------------------------------------
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_UP)) // up
  {
    keyNum = KN_UP;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_DOWN)) // down
  {
    keyNum = KN_DOWN;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_LEFT)) // left
  {
    keyNum = KN_LEFT;
  }
  else if (M5Cardputer.Keyboard.isKeyPressed(KEY_RIGHT)) // right
  {
    keyNum = KN_RIGHT;
  }
  else
  {
    return; // No relevant key pressed
  }

  changeSettings(settingMode, keyNum);
}

void changeSettings(SettingMode mode, KeyNum keyNo)
{
  if (mode == SM_ESC)
  {
    // clear L1 : setting display line
    canvas.fillRect(0, SC_LINES[1], X_WIDTH, H_CHR, TFT_BLACK);
  }

  else if (mode == SM_BRIGHT_LEVEL)
    changeBright(keyNo);

  else if (mode == SM_LOWBAT_THRESHOLD)
    changeLowBatThr(keyNo);

  else if (mode == SM_LANG)
    changeLang(keyNo);

  canvas.pushSprite(0, 0);
}

void changeLang(KeyNum keyNo)
{
  if (updateLang(keyNo))
  {
    wrtNVS(NVM_LANG, LANG_INDEX);
    dispMeasItem();
    dispBatItem();
  }
  prtSetting("lang = ", LANG[LANG_INDEX]);
}

bool updateLang(KeyNum keyNo)
{
  switch (keyNo)
  {
  case KN_UP:
  case KN_DOWN:
  case KN_RIGHT:
  case KN_LEFT:
    LANG_INDEX = (LANG_INDEX + 1) % (LANG_MAX + 1);
    return true; // Value changed
  default:
    break;
  }
  return false; // No change
}

void dispBatItem()
{
  canvas.fillRect(W_CHR * BATLVL_ITEM_POS, SC_LINES[0], W_CHR * BATLVL_ITEM_LEN, H_CHR, TFT_BLACK);
  canvas.setFont(&fonts::lgfxJapanMincho_16);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.drawString(BATLVL_TITLE[LANG_INDEX], W_CHR * BATLVL_ITEM_POS, SC_LINES[0]);
}

void dispMeasItem()
{
  canvas.setFont(&fonts::lgfxJapanGothic_24);
  canvas.setTextSize(1);
  int width = max(canvas.textWidth(meas_items[0]), canvas.textWidth(meas_items[1]));
  
  const int font_size = 24;

  // clear
  canvas.fillRect(0, SC_LINES[7], W_CHR * MEAS_ITEM_POS + width, font_size, TFT_BLACK);

  // measuremt items
  canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
  canvas.drawString(meas_items[LANG_INDEX], W_CHR * MEAS_ITEM_POS, SC_LINES[7]);
}

bool updateSettingValue(uint8_t &value, KeyNum keyNo, uint8_t min, uint8_t max, uint8_t step, uint8_t big_step)
{
  int tempValue = value;

  switch (keyNo)
  {
  case KN_UP:
    tempValue += big_step;
    break;
  case KN_DOWN:
    tempValue -= big_step;
    break;
  case KN_RIGHT:
    tempValue += step;
    break;
  case KN_LEFT:
    tempValue -= step;
    break;
  default:
    return false; // Not a value-changing key
  }

  // Clamp the value to the allowed range
  if (tempValue > max)
    tempValue = max;
  if (tempValue < min)
    tempValue = min;

  if (value != (uint8_t)tempValue)
  {
    value = (uint8_t)tempValue;
    return true; // Value changed
  }
  return false; // No change in value
}

void prtSetting(const char *msg, uint8_t data)
{
  char datBuf[4]; // message buffer
  snprintf(datBuf, sizeof(datBuf), "%3u", data);
  prtSetting(msg, datBuf);
}

void prtSetting(const char *msg, const char *data)
{
  // Line1 : setting display
  char msgBuf[31]; // message buffer
  snprintf(msgBuf, sizeof(msgBuf), "%s%s", msg, data);
  dbPrtln(msgBuf);

  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setFont(&fonts::lgfxJapanGothic_12);
  canvas.setTextSize(1);
  canvas.fillRect(0, SC_LINES[1], X_WIDTH, H_CHR, TFT_BLACK); // clear L1
  canvas.drawString(msgBuf, W_CHR * SETTING_DISP_POS, SC_LINES[1]);
}

void changeBright(KeyNum keyNo)
{
  const uint8_t step_short = 1;
  const uint8_t step_big = 10;
  if (updateSettingValue(BRIGHT_LVL, keyNo, BRIGHT_LVL_MIN, BRIGHT_LVL_MAX, step_short, step_big))
  {
    M5Cardputer.Display.setBrightness(BRIGHT_LVL);
    wrtNVS(NVM_BRIGHT, BRIGHT_LVL);
  }
  prtSetting("bright = ", BRIGHT_LVL);
}

void changeLowBatThr(KeyNum keyNo)
{
  const uint8_t step_short = 1;
  const uint8_t step_big = 10;
  if (updateSettingValue(LOWBAT_THRESHOLD, keyNo, LOWBAT_THRESHOLD_MIN, LOWBAT_THRESHOLD_MAX, step_short, step_big))
  {
    wrtNVS(NVM_LOWBAT, LOWBAT_THRESHOLD);
  }
  prtSetting("lowBattery threshold = ", LOWBAT_THRESHOLD);
}

void settingsInit()
{
  loadSetting(NVM_BRIGHT, BRIGHT_LVL, BRIGHT_LVL_INIT, BRIGHT_LVL_MIN, BRIGHT_LVL_MAX);
  M5Cardputer.Display.setBrightness(BRIGHT_LVL);
  loadSetting(NVM_LOWBAT, LOWBAT_THRESHOLD, LOWBAT_THRESHOLD_INIT, LOWBAT_THRESHOLD_MIN, LOWBAT_THRESHOLD_MAX);
  loadSetting(NVM_LANG, LANG_INDEX, LANG_INIT, 0, LANG_MAX);
}

static unsigned long PREV_BATCHK_TM = 0L;
static uint8_t PREV_BATLVL = 255; // Use an impossible value to force the first update
static bool batCheck_first = true;
#define BATLVL_FLUCTUATION 5                            // fluctuation
const unsigned long BATTERY_CHECK_INTERVAL_MS = 1993UL; // Interval for battery level check
void batteryState()
{
  unsigned long currentTime = millis(); // Get current time once

  if (currentTime - PREV_BATCHK_TM < BATTERY_CHECK_INTERVAL_MS)
    return;

  // This will update consecutiveLowBatteryCount
  PREV_BATCHK_TM = currentTime;
  uint8_t batLvl = (uint8_t)M5Cardputer.Power.getBatteryLevel(); // Get battery level
  dbPrtln("batLvl: " + String(batLvl));
  if (batLvl > BATLVL_MAX)
    batLvl = BATLVL_MAX;

  lowBatteryCheck(batLvl);

  if (batCheck_first)
  {
    batCheck_first = false;
  }
  else
  { // ** stable battery level is valid **
    if (abs(batLvl - PREV_BATLVL) > BATLVL_FLUCTUATION)
    {
      PREV_BATLVL = batLvl;
      return;
    }
  }

  PREV_BATLVL = batLvl;
  prtBatLvl(batLvl);
}

static uint8_t PREV_BATLVL_DISP = 255; // Use an impossible value to force the first update
void prtBatLvl(uint8_t batLvl)
{
  // Line0 : battery level display
  //---- 012345678901234567890123456789---
  // L0_"                      bat.xxx%"--

  if (batLvl == PREV_BATLVL_DISP)
    return;
  PREV_BATLVL_DISP = batLvl;

  char msg[4] = ""; // message buffer
  snprintf(msg, sizeof(msg), "%3u", batLvl);
  dbPrtln(msg);

  canvas.fillRect(W_CHR * BATLVL_VALUE_POS, SC_LINES[0], W_CHR * BATLVL_VALUE_LEN, H_CHR, TFT_BLACK); // clear
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setFont(&fonts::lgfxJapanMincho_16);
  canvas.setTextSize(1);
  canvas.drawString(msg, W_CHR * BATLVL_VALUE_POS, SC_LINES[0]);
  canvas.pushSprite(0, 0);
}

static uint8_t consecutiveLowBatteryCount = 0;
void lowBatteryCheck(uint8_t batLvl)
{
  const uint8_t LOWBAT_CONSECUTIVE_READINGS = 5;

  // Update consecutive low battery count
  if (batLvl < LOWBAT_THRESHOLD)
  {
    if (consecutiveLowBatteryCount < LOWBAT_CONSECUTIVE_READINGS)
    { // Avoid overflow if already at max
      consecutiveLowBatteryCount++;
    }
  }
  else
  {
    consecutiveLowBatteryCount = 0; // Reset if battery level is acceptable
    return;
  }

  if (consecutiveLowBatteryCount >= LOWBAT_CONSECUTIVE_READINGS)
  {
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_RED, TFT_BLACK);
    canvas.drawCenterString(F("Low Battery !!"), X_WIDTH / 2, SC_LINES[3], &fonts::Font4);
    canvas.pushSprite(0, 0);
    POWER_OFF();
    // *** NEVER RETURN ***
  }
}
