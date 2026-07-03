#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RTClib.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS3231 rtc;

// LED 핀
const int LED_HOUR = 8;
const int LED_MINUTE = 9;
const int LED_SECOND = 10;

// 버튼 핀
const int BTN_POWER = 2;
const int BTN_HOUR = 4;
const int BTN_MINUTE = 3;

// 시간 변화 감지용
int prevHour, prevMinute, prevSecond;

// 시/분 LED 2초 유지용
unsigned long hourLedOnTime = 0;
unsigned long minuteLedOnTime = 0;
bool hourLedActive = false;
bool minuteLedActive = false;

// 전원 버튼(토글)
bool prevPowerBtnState;
bool displayOn = true;
unsigned long lastPowerBtnTime = 0;

// 시+ 버튼
bool prevHourBtnState;
unsigned long lastHourBtnTime = 0;
int hourPressCount = 0; // 누른 횟수 기억

// 분+ 버튼
bool prevMinuteBtnState;
unsigned long lastMinuteBtnTime = 0;
int minutePressCount = 0; // 누른 횟수 기억

const unsigned long debounceDelay = 200;

void printCentered(String text, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int x = (SCREEN_WIDTH - w) / 2;
  display.setCursor(x, y);
  display.print(text);
}

void setup() {
  Serial.begin(9600);

  pinMode(LED_HOUR, OUTPUT);
  pinMode(LED_MINUTE, OUTPUT);
  pinMode(LED_SECOND, OUTPUT);

  pinMode(BTN_POWER, INPUT_PULLUP);
  pinMode(BTN_HOUR, INPUT_PULLUP);
  pinMode(BTN_MINUTE, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED 초기화 실패");
    while (1);
  }

  if (!rtc.begin()) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RTC not found");
    display.display();
    while (1);
  }

  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  DateTime now = rtc.now();
  prevHour = now.hour();
  prevMinute = now.minute();
  prevSecond = now.second();

  prevPowerBtnState = digitalRead(BTN_POWER);
  prevHourBtnState = digitalRead(BTN_HOUR);
  prevMinuteBtnState = digitalRead(BTN_MINUTE);
}

void loop() {
  DateTime now = rtc.now();
  unsigned long currentMillis = millis();

  // ---------- 초 LED: 매초 토글 ----------
  if (now.second() != prevSecond) {
    digitalWrite(LED_SECOND, !digitalRead(LED_SECOND));
    prevSecond = now.second();
  }

  // ---------- 분 LED: 분이 바뀌면 2초간 켜짐 ----------
  if (now.minute() != prevMinute) {
    digitalWrite(LED_MINUTE, HIGH);
    minuteLedActive = true;
    minuteLedOnTime = currentMillis;
    prevMinute = now.minute();
  }
  if (minuteLedActive && (currentMillis - minuteLedOnTime >= 2000)) {
    digitalWrite(LED_MINUTE, LOW);
    minuteLedActive = false;
  }

  // ---------- 시 LED: 시가 바뀌면 2초간 켜짐 ----------
  if (now.hour() != prevHour) {
    digitalWrite(LED_HOUR, HIGH);
    hourLedActive = true;
    hourLedOnTime = currentMillis;
    prevHour = now.hour();
  }
  if (hourLedActive && (currentMillis - hourLedOnTime >= 2000)) {
    digitalWrite(LED_HOUR, LOW);
    hourLedActive = false;
  }

  // ---------- 전원 버튼: 누른 만큼 되돌리기 + 카운트 초기화 + OLED 토글 ----------
  bool currentPowerBtnState = digitalRead(BTN_POWER);
  if (currentPowerBtnState == LOW && prevPowerBtnState == HIGH && (currentMillis - lastPowerBtnTime > debounceDelay)) {

    if (hourPressCount != 0 || minutePressCount != 0) {
      DateTime nowForRevert = rtc.now(); // 되돌리기 직전 최신 시각 다시 가져오기
      TimeSpan totalAdded(0, hourPressCount, minutePressCount, 0);
      DateTime reverted = nowForRevert - totalAdded;
      rtc.adjust(reverted);

      hourPressCount = 0;
      minutePressCount = 0;
    }

    displayOn = !displayOn;
    if (displayOn) {
      display.ssd1306_command(SSD1306_DISPLAYON);
    } else {
      display.ssd1306_command(SSD1306_DISPLAYOFF);
    }

    lastPowerBtnTime = currentMillis;
  }
  prevPowerBtnState = currentPowerBtnState;

  // ---------- 시+ 버튼: 시간 증가 + 카운트 누적 ----------
  bool currentHourBtnState = digitalRead(BTN_HOUR);
  if (currentHourBtnState == LOW && prevHourBtnState == HIGH && (currentMillis - lastHourBtnTime > debounceDelay)) {
    DateTime newTime = now + TimeSpan(0, 1, 0, 0);
    rtc.adjust(newTime);
    hourPressCount++;
    lastHourBtnTime = currentMillis;
  }
  prevHourBtnState = currentHourBtnState;

  // ---------- 분+ 버튼: 시간 증가 + 카운트 누적 ----------
  bool currentMinuteBtnState = digitalRead(BTN_MINUTE);
  if (currentMinuteBtnState == LOW && prevMinuteBtnState == HIGH && (currentMillis - lastMinuteBtnTime > debounceDelay)) {
    DateTime newTime = now + TimeSpan(0, 0, 1, 0);
    rtc.adjust(newTime);
    minutePressCount++;
    lastMinuteBtnTime = currentMillis;
  }
  prevMinuteBtnState = currentMinuteBtnState;

  // ---------- OLED 화면 갱신 ----------
  String dateStr = "";
  dateStr += now.year();
  dateStr += "/";
  if (now.month() < 10) dateStr += "0";
  dateStr += now.month();
  dateStr += "/";
  if (now.day() < 10) dateStr += "0";
  dateStr += now.day();

  String timeStr = "";
  if (now.hour() < 10) timeStr += "0";
  timeStr += now.hour();
  timeStr += ":";
  if (now.minute() < 10) timeStr += "0";
  timeStr += now.minute();
  timeStr += ":";
  if (now.second() < 10) timeStr += "0";
  timeStr += now.second();

  display.clearDisplay();
  printCentered(dateStr, 5);
  printCentered(timeStr, 30);
  display.display();
}