#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

WiFiMulti wifiMulti;

// Wifi name and password
const char* SSID = "TP-LINK_A158";
const char* PASSWORD = "80999805";

// Server IP and query params
// const String HOST = "http://192.168.4.102";
const String HOST = "http://192.168.0.108";
const String PORT = "6666";
const String MOD_ID = "7";
const String MODULE = "/?module=" + MOD_ID;

// ESP32 PIN config

// SPEED PINS
const uint8_t SPEED_PWM_PIN = 26;
const uint8_t SPEED_DIR_1 = 27;
const uint8_t SPEED_DIR_2 = 14;
const uint8_t SPEED_PWM_CHANNEL = 1;

// YAW PINS
const uint8_t YAW_PWM_PIN = 19;
const uint8_t YAW_DIR_1 = 23;
const uint8_t YAW_DIR_2 = 18;
const uint8_t YAW_PWM_CHANNEL = 2;

// PITCH PINS
const uint8_t PITCH_PWM_PIN = 2;
const uint8_t PITCH_DIR = 4;
const uint8_t PITCH_PWM_CHANNEL = 3;

// Time variable for checking last comm
unsigned long last_comm;

void setup() {
  // SPEED
  ledcAttachPin(SPEED_PWM_PIN, SPEED_PWM_CHANNEL);
  ledcSetup(SPEED_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(SPEED_DIR_1, OUTPUT);
  pinMode(SPEED_DIR_2, OUTPUT);

  // YAW
  ledcAttachPin(YAW_PWM_PIN, YAW_PWM_CHANNEL);
  ledcSetup(YAW_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(YAW_DIR_1, OUTPUT);
  pinMode(YAW_DIR_2, OUTPUT);

  // PITCH
  ledcAttachPin(PITCH_PWM_PIN, PITCH_PWM_CHANNEL);
  ledcSetup(PITCH_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(PITCH_DIR, OUTPUT);

  Serial.begin(115200);
  Serial.println();

  for(uint8_t t = 4; t > 0; t--) {
    Serial.print("[SETUP] WAIT ");
    Serial.println(t);
    Serial.flush();
    delay(500);
  }
  wifiMulti.addAP(SSID, PASSWORD);

  // Set up tasks to run independently.
  xTaskCreatePinnedToCore(
    checkIfCommFailed,
    "check_if_comm_failed", // A name just for humans
    1024, // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    2, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL,
    ARDUINO_RUNNING_CORE);
}

void loop() {
  // wait for WiFi connection
  if(wifiMulti.run() != WL_CONNECTED) {
    Serial.println("WiFi connection failed");
    stop_motors();
    return;
  }
  HTTPClient http;
  Serial.println("[HTTP] begin...");
  http.begin(HOST + String(":") + PORT + MODULE);
  Serial.println("[HTTP] GET...");
  int httpCode = http.GET();
  if(httpCode != HTTP_CODE_OK) {
    Serial.print("[HTTP] GET... failed, error: ");
    Serial.println(http.errorToString(httpCode).c_str());
    http.end();
    stop_motors();
    return;
  }

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& response_msg = jsonBuffer.parseObject(http.getString());

  if(response_msg.success()) {
    float speed = response_msg["speed"];
    float yaw = response_msg["yaw"];
    float pitch = response_msg["pitch"];
    write_pwm_speed(speed);
    write_pwm_yaw(yaw);
    write_pwm_pitch(pitch);
  }
  else {
    Serial.println("parseObject() failed");
  }
  http.end();
  last_comm = millis();
  delay(10);
}

void write_pwm_speed(float pwm) {
  bool dir = pwm > 1 ? LOW : HIGH;
  Serial.println(dir);
  digitalWrite(SPEED_DIR_1, dir);
  digitalWrite(SPEED_DIR_2, !dir);
  ledcWrite(SPEED_PWM_CHANNEL, abs(pwm));
  Serial.println(abs(pwm));
}

void write_pwm_yaw(float pwm) {
  bool dir = pwm > 1 ? LOW : HIGH;
  Serial.println(dir);
  digitalWrite(YAW_DIR_1, dir);
  digitalWrite(YAW_DIR_2, !dir);
  ledcWrite(YAW_PWM_CHANNEL, abs(pwm));
  Serial.println(abs(pwm));
}

void write_pwm_pitch(float pwm) {
  bool dir = pwm > 1 ? LOW : HIGH;
  Serial.println(dir);
  digitalWrite(PITCH_DIR, dir);
  ledcWrite(PITCH_PWM_CHANNEL, abs(pwm));
  Serial.println(abs(pwm));
}

void stop_motors() {
  ledcWrite(SPEED_PWM_CHANNEL, 0);
  ledcWrite(PITCH_PWM_CHANNEL, 0);
  ledcWrite(YAW_PWM_CHANNEL, 0);
}

void checkIfCommFailed(void *pvParameters){
  if (millis() - last_comm > 1000){
    Serial.println("WiFi Communication failed. Stopping motors");
    stop_motors();
  }
  vTaskDelay(1000);
}
