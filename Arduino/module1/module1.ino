#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

WiFiMulti wifiMulti;

// Wifi name and password
// const char* SSID = "Connectify-me";
// const char* PASSWORD = "hola1234";
const char* SSID = "INFINITUM689C_2.4";
const char* PASSWORD = "9rUK2YTmQX";

// Server IP and query params
const String HOST = "http://192.168.1.136";
const String PORT = "6666";
const String MODULE = "/?module=1";

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

  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println();

  for(uint8_t t = 4; t > 0; t--) {
    Serial.print("[SETUP] WAIT ");
    Serial.println(t);
    Serial.flush();
    delay(1000);
  }
  wifiMulti.addAP(SSID, PASSWORD);
}

void loop() {
  // wait for WiFi connection
  if(wifiMulti.run() != WL_CONNECTED) {
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
  }
  else {
    Serial.println("parseObject() failed");
  }
  http.end();
  delay(1);
}

void write_pwm_speed(float pwm) {
  bool dir = pwm > 1 ? LOW : HIGH;
  Serial.println(dir);
  ledcWrite(SPEED_PWM_CHANNEL, abs(pwm));
  digitalWrite(SPEED_DIR_1, dir);
  digitalWrite(SPEED_DIR_2, !dir);
  Serial.println(abs(pwm));
}

void write_pwm_yaw(float pwm) {
  bool dir = pwm > 1 ? LOW : HIGH;
  Serial.println(dir);
  ledcWrite(YAW_PWM_CHANNEL, abs(pwm));
  digitalWrite(YAW_DIR_1, dir);
  digitalWrite(YAW_DIR_2, !dir);
  Serial.println(abs(pwm));
}
