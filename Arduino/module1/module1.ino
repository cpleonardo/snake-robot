/*
 *  This sketch sends random data over UDP on a ESP32 device
 *
 */
// #include <WiFi.h>
// #include <WiFiUdp.h>
// #include <ArduinoJson.h>
//
//
// // WiFi network name and password:
// const char * networkName = "ARRIS-2152";
// const char * networkPswd = "848F706B6CFC6EAE";
//
// //IP address to send UDP data to:
// // either use the ip address of the server or
// // a network broadcast address
// const char * udpAddress = "192.168.0.6";
// const int udpPort = 3333;
//
// //Are we currently connected?
// boolean connected = false;
//
// //The udp library class
// WiFiUDP udp;
//
// void setup(){
//   // Initilize hardware serial:
//   Serial.begin(115200);
//
//   //Connect to the WiFi network
//   connectToWiFi(networkName, networkPswd);
// }
//
// void loop(){
//   // Allocate JsonBuffer
//   // Use arduinojson.org/assistant to compute the capacity.
//   StaticJsonBuffer<500> jsonBuffer;
//
//   // Create the root object
//   JsonObject& root = jsonBuffer.createObject();
//
//   root["module"] = 1;
//
//   JsonObject& data = root.createNestedObject("data");
//   data["temp"] = 14.2;
//   data["cond"] = "cloudy";
//
//   //only send data when connected
//   if(connected){
//     //Send a packet
//     udp.beginPacket(udpAddress,udpPort);
//     root.printTo(udp);
//     // udp.printf("Seconds since boot: %u", millis()/1000);
//     udp.println();
//     udp.endPacket();
//   }
//   //Wait for 1 second
//   delay(1000);
// }
//
// void connectToWiFi(const char * ssid, const char * pwd){
//   Serial.println("Connecting to WiFi network: " + String(ssid));
//
//   // delete old config
//   WiFi.disconnect(true);
//   //register event handler
//   WiFi.onEvent(WiFiEvent);
//
//   //Initiate connection
//   WiFi.begin(ssid, pwd);
//
//   Serial.println("Waiting for WIFI connection...");
// }
//
// //wifi event handler
// void WiFiEvent(WiFiEvent_t event){
//     switch(event) {
//       case SYSTEM_EVENT_STA_GOT_IP:
//           //When connected set
//           Serial.print("WiFi connected! IP address: ");
//           Serial.println(WiFi.localIP());
//           //initializes the UDP state
//           //This initializes the transfer buffer
//           udp.begin(WiFi.localIP(),udpPort);
//           connected = true;
//           break;
//       case SYSTEM_EVENT_STA_DISCONNECTED:
//           Serial.println("WiFi lost connection");
//           connected = false;
//           break;
//     }
// }

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "ARRIS-2152";
const char* password =  "848F706B6CFC6EAE";

void setup() {

  Serial.begin(115200);
  delay(4000);   //Delay needed before calling the WiFi.begin

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) { //Check for the connection
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");

}

void loop() {

 if(WiFi.status()== WL_CONNECTED){   //Check WiFi connection status

   HTTPClient http;

   http.begin("http://192.168.0.6:3333");
   http.addHeader("Content-Type", "application/json");

   int httpResponseCode = http.POST("POSTING from ESP32");

   if(httpResponseCode>0){
     String response = http.getString();
     Serial.println(httpResponseCode);
     Serial.println(response);

   }else{
     Serial.print("Error on sending POST: ");
     Serial.println(httpResponseCode);
   }

   http.end();  //Free resources

 }else{
   Serial.println("Error in WiFi connection");

 }

  delay(10000);  //Send a request every 10 seconds

}
