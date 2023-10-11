#include <numeric>
#include <vector>
#include <queue>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <WiFi.h>
#include <ctime>
#include <queue>
#include "secrets.h"
#include <ESP32Ping.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
 
 
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial 

int ledStatus = LOW;
// Set your new MAC Address
uint8_t newMACAddress[] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x05};

uint32_t lastReconnectAttempt = 0;
uint16_t combinedhex = 0;
uint16_t lastreceived;

std::queue<float> temp = {};     // queue for temperature values 
std::queue<float> conduct = {};  // queue for conductivity values
std::queue<float> pH = {};       // queue for pH values

float temp_now = 0;
float con_now = 0;
float pH_now = 0;
String MAC_now;

uint16_t HEX_A;
uint16_t HEX_B;
bool is_send_data = false;

String formattedDate;
String daystamp;
String timestamp;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"sg.pool.ntp.org",0);

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// RTC
struct tm timeinfo;
float timezone;
ESP32Time rtc;
uint64_t seconds, minutes, hours, days, months, year;




/**
 * @brief Returns the current datetime in a formatted string.
 */
String get_formatted_time(){
  return timeClient.getFormattedDate();
  // return rtc.getTime("%d %B %Y %I:%M:%S %p");
}


void hexconcat(uint16_t HEX_A, uint16_t HEX_B){
  combinedhex = HEX_A;
  combinedhex = combinedhex * 25;
  combinedhex = combinedhex | HEX_B;
  
}

void mqttPublish(String timestamp, String MAC_now, float temp_now, float con_now, float pH_now)
{
  StaticJsonDocument<200> doc;
  doc["timestamp"] = timestamp;
  doc["MAC"] = MAC_now;
  doc["temperature"] = temp_now;
  doc["conductivity"] = con_now;
  doc["pH"] = pH_now;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  client.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}


void mqttCallback(char *topic, byte *payload, unsigned int len) {

    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();

    // Only proceed if incoming message's topic matches
    if (String(topic) == AWS_IOT_SUBSCRIBE_TOPIC) {
        client.publish(AWS_IOT_PUBLISH_TOPIC, "Message Received");

    }
}


 
void connectAWS() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());
  Serial.println(WiFi.macAddress());
 
  // Configure WiFiClientSecure to use the AWS IoT device credentials
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);
 
  // Connect to the MQTT broker on the AWS endpoint we defined earlier
  client.setServer(AWS_IOT_ENDPOINT, 8883);
 
  // Create a message handler
  client.setCallback(mqttCallback);
 
  Serial.println("Connecting to AWS IOT");
 
  while (!client.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!client.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}
 



typedef struct struct_sensor_reading {
  String MAC;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
} struct_sensor_reading;

struct_sensor_reading incoming_data;

// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   Serial.println("Data Received");
//   memcpy(&incoming_data, incomingData, sizeof(incoming_data));
//   timestamp = get_formatted_time();
//   Serial.println(timestamp);
//   is_send_data = true;
// }
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("Data Received");
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  timestamp = get_formatted_time();
  lastreceived = millis();
  
  temp_now = incoming_data.temp;
  temp.push(temp_now);

  con_now = incoming_data.ECVal;
  conduct.push(con_now);

  pH_now = incoming_data.pHVal;
  pH.push(pH_now);

  MAC_now = incoming_data.MAC;
  
  is_send_data = true;
  
  
}
 
void setup() {
  Serial.begin(115200);
  connectAWS();

  // setupRTC();
  timeClient.begin();

  WiFi.enableLongRange(true);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Serial.print("[OLD] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // Serial.print("[NEW] ESP32 Board MAC Address:  ");
  // Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW initialized");
}
 
void loop(){
 
  if (is_send_data)
    {
    mqttPublish(timestamp, MAC_now, temp_now, con_now, pH_now);
    is_send_data = false;
    }
  
  // timestamp = get_formatted_time();
  // mqttPublish(timestamp,1,2,3,4,5,6,7);
  // Serial.println("Sample published");
  // delay(10000);

  client.loop();
  delay(1000);
}