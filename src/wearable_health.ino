#include <Wire.h>

/******** WiFi + ThingSpeak ********/
#include <WiFi.h>
#include <HTTPClient.h>

const char* WIFI_SSID = "MH - Orange";
const char* WIFI_PASS = "zyx@2025";

const char* THINGSPEAK_API_KEY = "6GSSVTVPFX4SAE87";
const char* THINGSPEAK_HOST = "http://api.thingspeak.com/update";

#define REPORTING_PERIOD_MS 15000
uint32_t lastSendMs = 0;

/******** MAX30105 ********/
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

/******** DS18B20 ********/
#include <OneWire.h>
#include <DallasTemperature.h>

/******** OLED ********/
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ================= OLED ================= */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

TwoWire I2C_OLED = TwoWire(1);   // OLED bus: SDA=16, SCL=17
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &I2C_OLED, OLED_RESET);

/* ================= DS18B20 ================= */
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensors(&oneWire);

float tempC = NAN;
uint32_t lastTempMs = 0;
#define TEMP_PERIOD_MS 1000

// Temperature calibration offset (+6 like you asked)
const float TEMP_OFFSET_C = 6.0;

/* ================= MAX30105 ================= */
MAX30105 particleSensor;

uint32_t irBuffer[100];
uint32_t redBuffer[100];
const int32_t bufferLength = 100;

int32_t spo2 = 0;
int8_t  validSPO2 = 0;

int32_t hr = 0;
int8_t  validHR = 0;

int hrAvg = 0;
int spO2Avg = 0;

/* Finger detection: dynamic baseline + delta */
uint32_t irBaseline = 0;               // measured at startup (no finger)
uint32_t FINGER_DELTA = 20000;         // start here; adjust if needed

// sanity limits
const int HR_MIN = 40;
const int HR_MAX = 200;
const int SPO2_MIN = 70;
const int SPO2_MAX = 100;

/* ================= UI timing ================= */
uint32_t lastOledMs = 0;
#define OLED_PERIOD_MS 250

/* ------------ WiFi ------------ */
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connecting to WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000) {
    Serial.print(".");
    delay(400);
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("✅ WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("⚠️ WiFi not connected. (Will keep trying in loop)");
  }
}

void ensureWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
}

/* ------------ ThingSpeak send ------------ */
void sendToThingSpeak(int hrSend, int spo2Send, float tempSend) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("⚠️ Skip ThingSpeak: WiFi not connected.");
    return;
  }

  HTTPClient http;
  String url = String(THINGSPEAK_HOST)
             + "?api_key=" + THINGSPEAK_API_KEY
             + "&field1=" + String(hrSend)
             + "&field2=" + String(spo2Send)
             + "&field3=" + String(tempSend, 2);

  http.begin(url);
  int code = http.GET();
  String payload = http.getString();
  http.end();

  Serial.print("📡 ThingSpeak HTTP: ");
  Serial.print(code);
  Serial.print(" | Response: ");
  Serial.println(payload);
}

/* ------------ MAX30105 init ------------ */
void initMAX30105() {
  Serial.println("Initializing MAX30105...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("❌ MAX30105 not found. Check wiring/power/SDA/SCL.");
    while (1) delay(1000);
  }

  // Stable settings (good for SpO2 + Maxim HR)
  byte ledBrightness = 80;
  byte sampleAverage = 8;
  byte ledMode = 2;       // Red + IR
  byte sampleRate = 200;  // 200/8 ~ 25 effective sps
  int  pulseWidth = 411;  // best resolution
  int  adcRange = 4096;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);

  particleSensor.setPulseAmplitudeRed(0x2F);
  particleSensor.setPulseAmplitudeIR(0x2F);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("✅ MAX30105 ready.");
}

/* Measure baseline IR with NO finger */
void measureIRBaseline() {
  Serial.println("➡️ Remove finger... measuring IR baseline (3 seconds)...");
  uint64_t sum = 0;
  int count = 0;

  uint32_t start = millis();
  while (millis() - start < 3000) {
    particleSensor.check();
    if (particleSensor.available()) {
      sum += particleSensor.getIR();
      particleSensor.nextSample();
      count++;
    }
    delay(5);
  }

  if (count == 0) {
    irBaseline = 0;
    Serial.println("⚠️ Could not read IR for baseline.");
  } else {
    irBaseline = sum / (uint32_t)count;
    Serial.print("✅ IR baseline = ");
    Serial.println(irBaseline);
  }

  Serial.println("➡️ Now place finger on sensor...");
}

/* Fill initial buffer once */
void fillInitialBuffer() {
  Serial.println("Filling initial buffer (100 samples)...");
  for (int i = 0; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
  Serial.println("Initial buffer filled ✅");
}

void computeHRSpO2() {
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &hr, &validHR
  );
}

void shiftAndRead25() {
  for (int i = 25; i < 100; i++) {
    redBuffer[i - 25] = redBuffer[i];
    irBuffer[i - 25]  = irBuffer[i];
  }

  for (int i = 75; i < 100; i++) {
    while (!particleSensor.available()) particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }
}

bool fingerDetected(uint32_t lastIR) {
  if (irBaseline == 0) return false;
  return (lastIR > (irBaseline + FINGER_DELTA));
}

void smoothSignals(bool fingerOn) {
  if (!fingerOn) {
    hrAvg = 0;
    spO2Avg = 0;
    return;
  }

  if (validHR == 1 && hr >= HR_MIN && hr <= HR_MAX) {
    hrAvg = (hrAvg == 0) ? hr : (hrAvg + hr) / 2;
  }

  if (validSPO2 == 1 && spo2 >= SPO2_MIN && spo2 <= SPO2_MAX) {
    spO2Avg = (spO2Avg == 0) ? spo2 : (spO2Avg + spo2) / 2;
  }
}

/* ================= Setup ================= */
void setup() {
  Serial.begin(115200);
  delay(500);

  // WiFi
  connectWiFi();

  // I2C for MAX30105: SDA=21, SCL=22
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // OLED on second I2C: SDA=16, SCL=17
  I2C_OLED.begin(16, 17);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("❌ OLED not found (addr 0x3C?)");
    while (1) delay(1000);
  }
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);

  // Temp
  tempSensors.begin();

  // MAX30105 init + baseline
  initMAX30105();
  measureIRBaseline();

  // Prepare HR/SpO2 buffers
  fillInitialBuffer();
  computeHRSpO2();

  display.setCursor(0, 0);
  display.println("System Ready");
  display.display();
  delay(1000);

  Serial.println("✅ Running...");
  Serial.print("FINGER_DELTA = ");
  Serial.println(FINGER_DELTA);
}

/* ================= Loop ================= */
void loop() {
  // keep wifi alive
  ensureWiFi();

  // update MAX30105 data window
  shiftAndRead25();
  computeHRSpO2();

  uint32_t lastIR = irBuffer[99];
  bool fingerOn = fingerDetected(lastIR);
  smoothSignals(fingerOn);

  // Temperature (every 1 second)
  if (millis() - lastTempMs >= TEMP_PERIOD_MS) {
    tempSensors.requestTemperatures();
    tempC = tempSensors.getTempCByIndex(0);
    lastTempMs = millis();
  }

  float tempDisplay = (isnan(tempC) || tempC < -100) ? NAN : (tempC + TEMP_OFFSET_C);

  // Serial debug
  Serial.print("Finger=");
  Serial.print(fingerOn ? "ON " : "OFF");
  Serial.print(" IR=");
  Serial.print(lastIR);
  Serial.print(" base=");
  Serial.print(irBaseline);
  Serial.print(" | HR=");
  Serial.print(hrAvg);
  Serial.print(" | SpO2=");
  Serial.print(spO2Avg);
  Serial.print(" | Temp=");
  if (isnan(tempDisplay)) Serial.println("N/A");
  else { Serial.print(tempDisplay, 2); Serial.println(" C"); }

  // OLED
  if (millis() - lastOledMs >= OLED_PERIOD_MS) {
    display.clearDisplay();

    display.setCursor(0, 0);
    display.print("Finger: ");
    display.print(fingerOn ? "ON" : "OFF");

    display.setCursor(0, 12);
    display.print("HR: ");
    display.print(hrAvg);

    display.setCursor(64, 12);
    display.print("SpO2: ");
    display.print(spO2Avg);

    display.setCursor(0, 28);
    display.print("Temp: ");
    if (isnan(tempDisplay)) display.print("N/A");
    else display.print(tempDisplay, 1);
    display.print(" C");

    display.display();
    lastOledMs = millis();
  }

  // ThingSpeak (every 15 seconds)
  if (millis() - lastSendMs >= REPORTING_PERIOD_MS) {
    int hrSend = fingerOn ? hrAvg : 0;
    int spSend = fingerOn ? spO2Avg : 0;

    // If temp invalid, send -127 (common DS18B20 error marker)
    float tSend = isnan(tempDisplay) ? -127.0 : tempDisplay;

    sendToThingSpeak(hrSend, spSend, tSend);
    lastSendMs = millis();
  }

  delay(5);
}
