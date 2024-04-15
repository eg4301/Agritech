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

#include <SoftwareSerial.h>
#include <Sol16_RS485.h>

#include "esp_adc_cal.h"

 
#define AWS_IOT_PUBLISH_TOPIC   "Soil1/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial 

int ledStatus = LOW;
// Set your new MAC Address
uint8_t newMACAddress[] = {0x40, 0x22, 0xD8, 0x66, 0x7B, 0xF0};

uint32_t lastReconnectAttempt = 0;
uint16_t combinedhex = 0;
uint16_t lastreceived;

// std::queue<float> temp = {};     // queue for temperature values 
// std::queue<float> conduct = {};  // queue for conductivity values
// std::queue<float> pH = {};       // queue for pH values

float temp_now = 0;
float con_now = 0;
float pH_now = 0;
float atmtemp_now = 0;
float hum_now = 0;
float CO2_now = 0;
float oxy_now = 0;
float N_now = 0;
float P_now = 0;
float K_now = 0;
int MAC_now;
float sample_temp,sample_pH,sample_ec = 0;

uint16_t HEX_A;
uint16_t HEX_B;
bool is_send_data = false;
bool is_sampling_data = false;
bool is_fertilize = false;
bool is_buffer = false;

String formattedDate;
String daystamp;
String timestamp;

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP,"pool.ntp.org",0);

WiFiClientSecure net = WiFiClientSecure();
PubSubClient client(net);

// RTC
struct tm timeinfo;
float timezone;
ESP32Time rtc;
uint64_t seconds, minutes, hours, days, months, year;

//OTA
const char* host = "esp32";

//WiFi Reconnect
unsigned long previousMillis = 0;
unsigned long interval = 30000;

// Fertilizer Timer
unsigned long fertilizeMillis = 0;
unsigned long fert_timer = 900000;

unsigned long buffMillis = 0;
unsigned long sampleMillis = 0;

// AWS Reconnect
long lastReconnectAttempt1;

// Actuation system
float chamber_volume = 0.2;           // in dm3 (cubic decimeters) aka litres
float tank_volume = 30;               // in dm3 (cubic decimeters) aka litres
float sol_A_N_conc = 21;              // in g/dm3
float sol_B_N_conc = 5;               // in g/dm3
float N_temp_store = 0;               // in g/dm3
float soil_to_tank_transform = 0.67;  // Accounts for number of tank fills to distribute to entire patch
float chamber_counter = 0;            // Keeps track of number of sampling chamber fills required per tank
float tank_counter = 0;               // Keeps track of number of tank fills required to dose the patch
float N_mass_tank = 0;                // Tank nitrogen mass counter
float N_mass_required = 0;            // Required Nitrogen Mass
float fert_volume = 0;
float res_chamber = 0;



// Thresholds
float pH_desired = 0;
float N_desired = 0;
float EC_desired = 0;
boolean updateThreshold = false;

#define baud_rate 9600

// RS485 pins in use
#define RX_PIN 12    // Soft Serial Receive pin, connected to RO //  
#define TX_PIN 14    // Soft Serial Transmit pin, connected to DI // 
#define CTRL_PIN 13  // RS485 Direction control, connected to RE and DE // 


// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

// Declarations for Actuation (pumps + valve)
#define HIGH_PERISTALTIC_PIN_1 17   //Relay 2
#define HIGH_PERISTALTIC_PIN_2 18   //Relay 1 (pumps water opposite direction)
#define PERISTALTIC_PIN_1 7         //Relay 5
#define PERISTALTIC_PIN_2 15        //Relay 4 [Buffer]
#define PERISTALTIC_PIN_3 16        //Relay 3 [Fertilizer]
#define RECIRCULATING_PUMP 6        //Relay 6
#define IRRIGATION_PUMP 5           //Relay 7
#define WATER_VALVE 4               //Relay 8

// Declarations for Water Switch:
#define WATER_SWITCH_PIN 8

// Define length of time pumps and valves are open

#define PUMP_DURATION 120000        // Time used to pump clean water in ms
#define HIGH_PUMP_DURATION 30000    // Time used to pump sample in ms
#define CLEAR_DURATION 30000        // Time used to clear sample in ms
#define MIXING_DURATION 60000       // Time used to mix sample in ms
#define IRRIGATION_DURATION 120000  // Time used to pump sample in ms
#define FERT_DURATION 60000         // Time used to pump fertilizer into mixing tank in ms
#define BUFF_DURATION 60000         // Time used to pump buffer into mixing tank in ms
#define SAMPLE_DURATION 900000      // Time between sampling chamber runs in ms



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

void mqttPublish(String timestamp, int MAC_now, float temp_now, float con_now, float pH_now, float atmtemp_now, float hum_now, float CO2_now, float oxy_now, float N_now, float P_now, float K_now)
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
  doc["Nitrogen"] = N_now;
  doc["Phosphorous"] = P_now;
  doc["Potassium"] = K_now;
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

    StaticJsonDocument<300> doc;

    auto error = deserializeJson(doc,payload);
    

    if(error){
      Serial.println("Message parsing failed");
      client.publish(AWS_IOT_PUBLISH_TOPIC, "Threshold failed to parse");
    }

    if (updateThreshold && !error) {
      Serial.println("Message Parsed, updating Thresholds...");
      Serial.print("Previous pH Threshold: ");
      Serial.println(pH_desired);
      pH_desired = doc["pH"];
      Serial.print("Current pH Threshold: ");
      Serial.println(pH_desired);
      Serial.print("Previous EC Threshold: ");
      Serial.println(EC_desired);
      EC_desired = doc["EC"];
      Serial.print("Current EC Threshold: ");
      Serial.println(EC_desired);
      Serial.print("Previous Nitrogen Threshold: ");
      Serial.println(N_desired);
      N_desired = doc["Nitrogen"];
      Serial.print("Current Nitrogen Threshold: ");
      Serial.println(N_desired);
      Serial.println("Thresholds updated");
    }

    // String messageTemp;
  
    // for (int i = 0; i < len; i++) {
    //   Serial.print((char)payload[i]);
    //   messageTemp += (char)payload[i];
    // }
    // Serial.println();

    // // Only proceed if incoming message's topic matches
    // if (String(topic) == AWS_IOT_SUBSCRIBE_TOPIC) {
    //   client.publish(AWS_IOT_PUBLISH_TOPIC, "Thresholds Received");
    //   if (updateThreshold){
    //     pH_desired = messageTemp[3];
    //     EC_desired = messageTemp[4];
    //     N_desired = messageTemp[5];
    //     SerialMon.print("Thresholds updated");
    //     client.publish(AWS_IOT_PUBLISH_TOPIC, "Thresholds Updated");
    //   }
    // }
    


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
  int MAC;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
  float atmtemp = 0;
  float hum = 0;
  float CO2 = 0;
  float Oxy = 0;
  float N = 0;
  float P = 0;
  float K = 0;
} struct_sensor_reading;

struct_sensor_reading incoming_data;

typedef struct struct_sample_reading {
  int MAC;
  float sam_pH = 0;
  float sam_EC = 0;
  float sam_Temp = 0;
} struct_sample_reading;

struct_sample_reading samplingData;
struct_sample_reading myData;

// void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
//   Serial.println("Data Received");
//   memcpy(&incoming_data, incomingData, sizeof(incoming_data));
//   timestamp = get_formatted_time();
//   Serial.println(timestamp);
//   is_send_data = true;
// }

void checkThreshold(){
  if (sample_ec < EC_desired){
    is_fertilize = true;
    Serial.println("Need to fertilize!");
  }

  if (sample_pH < pH_desired){
    is_buffer = true;
    Serial.println("Need to add Buffer!");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  int RecMAC = *mac;
  Serial.println("Data Received from: ");
  Serial.println(RecMAC);
  if (len > 10){
    memcpy(&incoming_data, incomingData, sizeof(incoming_data));
    timestamp = get_formatted_time();
    lastreceived = millis();

    temp_now = incoming_data.temp;
    con_now = incoming_data.ECVal;
    pH_now = incoming_data.pHVal;
    MAC_now = incoming_data.MAC;
    atmtemp_now = incoming_data.atmtemp;
    hum_now = incoming_data.hum;
    CO2_now = incoming_data.CO2;
    oxy_now = incoming_data.Oxy;
    N_now = incoming_data.N;
    P_now = incoming_data.P;
    K_now = incoming_data.K;

    is_send_data = true;}  

  if (len < 10){
    memcpy(&samplingData, incomingData,sizeof(samplingData));
    sample_ec = samplingData.sam_EC;
    sample_pH = samplingData.sam_pH;
    sample_temp = samplingData.sam_Temp;
    checkThreshold();
  }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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


void fert_seq(){
  Serial.println("Starting Fertilisation Sequence");
  digitalWrite(PERISTALTIC_PIN_3,HIGH);
  Serial.print("Fertilizing for: ");
  Serial.print(FERT_DURATION);
  Serial.println(" ms");
  delay(FERT_DURATION);
  digitalWrite(PERISTALTIC_PIN_3,LOW);
  Serial.println("Fertilising Complete");
}

void buff_seq(){
  Serial.println("Starting Buffer adding Sequence");
  digitalWrite(PERISTALTIC_PIN_2,HIGH);
  Serial.print("Adding Buffer for: ");
  Serial.print(BUFF_DURATION);
  Serial.println(" ms");
  delay(BUFF_DURATION);
  digitalWrite(PERISTALTIC_PIN_2,LOW);
  Serial.println("Buffer adding Complete");
}

esp_now_peer_info_t peerInfo;


WebServer server(80);

/*
 * Login page
 */
const char* loginIndex = 
 "<form name='User Login'>"
    "<table width='20%' bgcolor='818352' align='center'>"
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
    "if(form.userid.value=='admin' && form.pwd.value=='agr1tech!')"
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

/*
 * setup function
 */


void setup() {
  Serial.begin(9600);

  // Initialize pins
  pinMode(HIGH_PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_2, OUTPUT);
  pinMode(PERISTALTIC_PIN_3, OUTPUT);
  pinMode(RECIRCULATING_PUMP, OUTPUT);
  pinMode(IRRIGATION_PUMP, OUTPUT);
  pinMode(WATER_VALVE, OUTPUT);
  pinMode(HIGH_PERISTALTIC_PIN_2, OUTPUT);

  pinMode(WATER_SWITCH_PIN, INPUT);


  connectAWS();

  // setupRTC();
  timeClient.begin();
  timeClient.update();
  timeClient.setTimeOffset(28800);

  WiFi.enableLongRange(true);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Serial.print("[OLD] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // Serial.print("[NEW] ESP32 Board MAC Address:  ");
  // Serial.println(WiFi.macAddress());
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
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  Serial.println("ESP NOW Initialized");

  // Register peer
  memcpy(peerInfo.peer_addr, newMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer Added");
  }

  digitalWrite(RECIRCULATING_PUMP, HIGH);
}
 
void loop(){
  unsigned long currentMillis = millis();
  
  
    
  // Put ESP to deep sleep every 12h
  if (millis() >= 43200000) {
    esp_sleep_enable_timer_wakeup(20e6);
  }

  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >=interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
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

  if (currentMillis - sampleMillis > SAMPLE_DURATION){
    esp_err_t result = esp_now_send(0, (uint8_t*) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      is_send_data = false;
    }
    else {
      Serial.println("Error sending the data");
    }
  }

  if (is_send_data)
    {
    mqttPublish(timestamp, MAC_now, temp_now, con_now, pH_now, atmtemp_now, hum_now, CO2_now, oxy_now, N_now, P_now, K_now);
    is_send_data = false;
    }

  
  
  if (is_fertilize && (currentMillis - fertilizeMillis >= fert_timer)){
    fert_seq();
    fertilizeMillis = currentMillis;
    is_fertilize = false;
  }

  if (is_buffer && (currentMillis - buffMillis >= fert_timer)) {
    buff_seq();
    buffMillis = currentMillis;
    is_buffer = false;
  }

  client.loop();
  server.handleClient();
  esp_now_register_send_cb(OnDataSent);
  delay(1000);
}