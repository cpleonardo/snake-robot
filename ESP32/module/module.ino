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
const String HOST = "http://192.168.0.108";
const String PORT = "6666";
const String MOD_ID = "1";
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

// ENCODER SPEED PINS
const uint8_t SPEED_ENCODER_A = 12;
const uint8_t SPEED_ENCODER_B = 13;

// ENCODER PITCH PINS
const uint8_t PITCH_ENCODER_A = 16;
const uint8_t PITCH_ENCODER_B = 17;

// ENCODER YAW PINS
const uint8_t YAW_ENCODER_A = 34;
const uint8_t YAW_ENCODER_B = 35;

// Encoder pulse counter variables
volatile double yaw_counter = 0;
volatile double pitch_counter = 0;

// Sample time
const uint8_t SAMPLE_TIME = 10;

// Desired pitch and yaw angles
float desired_pitch_angle = 0;
float desired_yaw_angle = 0;


void setup() {
  // SPEED
  ledcAttachPin(SPEED_PWM_PIN, SPEED_PWM_CHANNEL);
  ledcSetup(SPEED_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(SPEED_DIR_1, OUTPUT);
  pinMode(SPEED_DIR_2, OUTPUT);
  pinMode(SPEED_ENCODER_A, INPUT);
  pinMode(SPEED_ENCODER_B, INPUT);

  // YAW
  ledcAttachPin(YAW_PWM_PIN, YAW_PWM_CHANNEL);
  ledcSetup(YAW_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(YAW_DIR_1, OUTPUT);
  pinMode(YAW_DIR_2, OUTPUT);
  pinMode(YAW_ENCODER_A, INPUT);
  pinMode(YAW_ENCODER_B, INPUT);

  // PITCH
  ledcAttachPin(PITCH_PWM_PIN, PITCH_PWM_CHANNEL);
  ledcSetup(PITCH_PWM_CHANNEL, 12000, 8); // 12 kHz PWM, 8-bit resolution
  pinMode(PITCH_DIR, OUTPUT);
  pinMode(PITCH_ENCODER_A, INPUT);
  pinMode(PITCH_ENCODER_B, INPUT);

  Serial.begin(115200);
  Serial.println();

  for(uint8_t t = 4; t > 0; t--) {
    Serial.print("[SETUP] WAIT ");
    Serial.println(t);
    Serial.flush();
    delay(1000);
  }

  // Router connection
  wifiMulti.addAP(SSID, PASSWORD);

  // Interrupt routines
  attachInterrupt(
    digitalPinToInterrupt(SPEED_ENCODER_A), speedEncoderEvent, RISING);
  attachInterrupt(
    digitalPinToInterrupt(PITCH_ENCODER_A), pitchEncoderEvent, RISING);
  attachInterrupt(
    digitalPinToInterrupt(YAW_ENCODER_A), yawEncoderEvent, RISING);

  // Set up tasks to run independently.
  xTaskCreatePinnedToCore(
    pitchPID,
    "compute pitch PID", // A name just for humans
    8000, // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL,
    ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
    yawPID,
    "compute yaw PID", // A name just for humans
    8000, // This stack size can be checked & adjusted by reading the Stack Highwater
    NULL,
    1, // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    NULL,
    ARDUINO_RUNNING_CORE);
}

void loop() {
  // wait for WiFi connection
  if(wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection failed");
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
    desired_yaw_angle = response_msg["yaw"];
    desired_pitch_angle = response_msg["pitch"];
    write_pwm_pitch(response_msg["speed"]);
  }
  else {
    Serial.println("parseObject() failed");
  }
  http.end();
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

void speedEncoderEvent(){
  // do nothing by now
}

void yawEncoderEvent(){
  if(digitalRead(YAW_ENCODER_B))
    yaw_counter--;
  else
    yaw_counter++;
}

void pitchEncoderEvent(){
  if(digitalRead(PITCH_ENCODER_B))
    pitch_counter--;
  else
    pitch_counter++;
}

void yawPID(void *pvParameters){
  (void) pvParameters;
  const uint8_t K_P = 12;
  const uint8_t K_I = 2;
  float acc_error = 0;
  while(true){
    float current_angle = yaw_counter * 100 * 12 / (360 * 3);
    float error = desired_yaw_angle - current_angle;
    float integral_error = 0.001 * SAMPLE_TIME * (error - acc_error) / 2;
    float u = K_P * error + K_I * integral_error;
    acc_error = integral_error;
    write_pwm_yaw(u);
    Serial.print("Computed yaw");
    vTaskDelay(SAMPLE_TIME);
  }
}

void pitchPID(void *pvParameters){
  (void) pvParameters;
  const uint8_t K_P = 12;
  const uint8_t K_I = 2;
  float acc_error = 0;
  while(true){
    float current_angle = pitch_counter * 100 * 12 / (360 * 3);
    float error = desired_pitch_angle - current_angle;
    float integral_error = 0.001 * SAMPLE_TIME * (error - acc_error) / 2;
    float u = K_P * error + K_I * integral_error;
    acc_error = integral_error;
    write_pwm_pitch(u);
    Serial.print("Computed pitch");
    vTaskDelay(SAMPLE_TIME);
  }
}
