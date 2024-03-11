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

#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
 
 
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial 

int ledStatus = LOW;
// Set your new MAC Address
uint8_t newMACAddress[] = {0x0E, 0x10, 0x04, 0x03, 0x00, 0x01};

uint32_t lastReconnectAttempt = 0;
uint16_t combinedhex = 0;
uint16_t lastreceived;

// std::queue<float> temp = {};     // queue for temperature values 
// std::queue<float> conduct = {};  // queue for conductivity values
// std::queue<float> pH = {};       // queue for pH values

// Variables for logging data
float temp_now = 0;
float con_now = 0;
float pH_now = 0;
float atmtemp_now = 0;
float hum_now = 0;
float CO2_now = 0;
float oxy_now = 0;
int MAC_now = 0;

/*Variables for HEX Concetenation*/
uint16_t HEX_A;
uint16_t HEX_B;

/*Boolean to check if need to send data*/ 
bool is_send_data = false;

String formattedDate;
String daystamp;
String timestamp;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"pool.ntp.org",28800);

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// RTC
struct tm timeinfo;
float timezone;
ESP32Time rtc;
uint64_t seconds, minutes, hours, days, months, year;

<<<<<<< Updated upstream
// OTA
const char* host = "esp32";

// WiFi Reconnect
=======
>>>>>>> Stashed changes
unsigned long previousMillis = 0;
unsigned long interval = 30000;

// AWS Reconnect
long lastReconnectAttempt1;

/**
 * @brief Returns the current datetime in a formatted string.
 */
String get_formatted_time(){
  return timeClient.getFormattedDate();
  // return rtc.getTime("%d %B %Y %I:%M:%S %p");
}


void hexconcat(uint16_t HEX_A, uint16_t HEX_B){
  combinedhex = HEX_A;
  combinedhex = combinedhex * 256;
  combinedhex = combinedhex | HEX_B;
  
}

void mqttPublish(String timestamp, int MAC_now, float temp_now, float con_now, float pH_now, float atmtemp_now, float hum_now, float CO2_now, float oxy_now)
{
  StaticJsonDocument<200> doc;
  doc["timestamp"] = timestamp;
  doc["MAC"] = MAC_now;
  doc["temperature"] = temp_now;
  doc["conductivity"] = con_now;
  doc["pH"] = pH_now;
  doc["atm_temperature"] = atmtemp_now;
  doc["humidity"] = hum_now;
  doc["CO2"] = CO2_now;
  doc["O2"] = oxy_now;
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
    delay(1000);
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
 
boolean AWS_reconnect() {
  if (client.connect(THINGNAME)){
    // Test Publish to AWS_IOT_PUBLISH_TOPIC
    client.publish(AWS_IOT_PUBLISH_TOPIC, "Gateway successfully reconnected");

    // Resub to subscribe topic
    client.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
  }
  return client.connected();
}


typedef struct struct_sensor_reading {
  int MAC;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
  float atmtemp = 0;
  float hum = 0;
  float CO2 = 0;
  float Oxy = 0;
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
  // temp.push(temp_now);

  con_now = incoming_data.ECVal;
  // conduct.push(con_now);

  pH_now = incoming_data.pHVal;
  // pH.push(pH_now);

  MAC_now = incoming_data.MAC;

  atmtemp_now = incoming_data.atmtemp;

  hum_now = incoming_data.hum;

  CO2_now = incoming_data.CO2;

  oxy_now = incoming_data.Oxy;
  
  is_send_data = true;
  
  
}

WebServer server(80);

/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='EG4301' && form.pwd.value=='agritech.1')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";
 
void setup() {
  Serial.begin(115200);  
  Serial.print("[OLD] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  Serial.print("[NEW] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  
  lastReconnectAttempt1 = 0;

  connectAWS();

  // setupRTC();
  timeClient.begin();
  timeClient.update();
  timeClient.setTimeOffset(28800);

  WiFi.enableLongRange(true);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Start Web Server for OTA firmware flashing
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();

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
  unsigned long currentMillis = millis();
<<<<<<< Updated upstream

  // Put ESP to deep sleep every 12h
  if (millis() >= 43200000) {
    esp_sleep_enable_timer_wakeup(20e6);
  }

  // Update time client
  timeClient.update();
  delay(100);
  
  // if WiFi is down, try reconnecting
=======
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
>>>>>>> Stashed changes
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
<<<<<<< Updated upstream
    connectAWS();
    previousMillis = currentMillis;
  }

  if ((WiFi.status() == WL_CONNECTED) && (!client.connected())){
    long now = millis();
    Serial.println("Client disconnected from IoT Core");
    if (now - lastReconnectAttempt1 > 25000) {
      lastReconnectAttempt1 = now;
      if (AWS_reconnect()){
        lastReconnectAttempt1 = 0;
        Serial.println("Client successfully reconnected to IoT Core");
      }
    }
  }

 
=======
    WiFi.reconnect();
    previousMillis = currentMillis;
  }
>>>>>>> Stashed changes
  if (is_send_data)
    {
    mqttPublish(timestamp, MAC_now, temp_now, con_now, pH_now, atmtemp_now, hum_now, CO2_now, oxy_now);
    is_send_data = false;
    }
  
  // timestamp = get_formatted_time();
  // mqttPublish(timestamp,1,2,3,4,5,6,7);
  // Serial.println("Sample published");
  // delay(10000);

  client.loop();
  server.handleClient();
  delay(100);
}