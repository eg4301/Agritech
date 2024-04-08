#include <stdint.h>
#include <Arduino.h>


#include <Sol16_RS485.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include "driver/gpio.h"

#include "OneWire.h"
#include "DallasTemperature.h"

#define baud_rate 115200


// Declarations for pH Sensor:
#define PH_PIN 9             // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 10       // Flat deviation compensate
#define scaleOff_ph -3       // Scale deviation compensate
float avgRead_ph;             //Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value

// Declarations for EC Sensor:
#define EC_PIN 10
#define ecLow 750
#define ecHigh 7200
float ecValue;

// Declarations for Temp Sensor:
#define oneWireBus 11     
float temperature;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);
DeviceAddress tempDeviceAddress;
int numberOfDevices;

// Declarations for Water Pressure Sensor:
#define PRESSURE_PIN 12
#define pressureV_offset 0.5
#define container_area 0.158 //m^2
float waterAmt;

bool sampling = false;
bool is_send_data = false;

// Declarations for Peristaltic Pump and Valve
#define HIGH_PERISTALTIC_PIN_1 17  // Relay 2 (Fill Chamber)
#define HIGH_PERISTALTIC_PIN_2 18  // Relay 1 (Drain Chamber)
#define PERISTALTIC_PIN_3 16       // Relay 3 (Fill with reservoir water)

unsigned long EMPTY_DURATION = 40000;
unsigned long FILL_DURATION = 30000;

uint8_t broadcastAddress[] = {0x68, 0xB6, 0xB3, 0x51, 0xD3, 0x28};  // ! REPLACE WITH YOUR RECEIVER MAC Address



typedef struct struct_sensor_reading {
  int MAC ;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
} struct_sensor_reading;

struct_sensor_reading myData;

esp_now_peer_info_t peerInfo;

// Insert your SSID
constexpr char WIFI_SSID[] = "localize_project";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// Function definitions:
void phRead(){
  int j = 0;
  for(int i = 0; i<10; i++){
    j += analogRead(PH_PIN) * 3.3 / 4096.0;
  }
  pHValue = j/10;  
  pHValue = scaleOff_ph * pHValue + flatOff_ph;             // Transform avgRead_ph to get pHValue
  Serial.print(pHValue);
  Serial.println(" pH");
}

void ecRead(){
  float k = 0; 
  float V = 0; 
  float grad = 0;
  
  for(int i = 0; i<10; i++){
    k += analogRead(EC_PIN) * 3.3;
  }
  V = k/10;  

  grad = 11.467/(ecHigh - ecLow);
  ecValue = 12.88 - (ecHigh - V)*grad;

  Serial.print("EC V = ");
  Serial.println(V/4096);
  Serial.print("EC = ");
  Serial.print(ecValue);
  Serial.println("ms/cm^2");
}

void tempRead(){
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0);
  Serial.print(temperature);
  Serial.println("ºC");
}

void waterlevelRead(){
  float pressureV = 0;
  float k = 0;
  for(int i = 0; i<10; i++){
    k += analogRead(PRESSURE_PIN) * 3.3;
    delay(200);
  }
  pressureV = k/10;  
  Serial.print("Pressure V = ");
  Serial.println(pressureV/4096);
  //calculates amount of water in m^3
  //calculates pressure in kPa (density accounted for in waterAmt calculation)

  float pressure = (pressureV - pressureV_offset) * (1600 / (4.5 - pressureV_offset)) / 4096.0 ;
  waterAmt = pressure * container_area / 9.81;
  Serial.print(waterAmt);
  Serial.println("m^3");
}

void sampling_sequence(){
  ecRead();
  phRead();
  tempRead();

  myData.ECVal = ecValue;
  myData.pHVal = pHValue;
  myData.temp = temperature;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
  Serial.print(" ");
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  Serial.println("Data Received");
  is_send_data = true;
  sampling = true;
  Serial.println("Starting Sampling Sequence...");
}


void setup() {
  Serial.begin(baud_rate);

  Serial.println("Initializing...");
  delay(2000);
  sensors.begin();
  delay(2000);

  myData.MAC = 8;

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  int32_t channel = getWiFiChannel(WIFI_SSID);
  while (channel < 1) {
    delay(1000);
    Serial.println("WiFi Channel Not Found!");
    channel = getWiFiChannel(WIFI_SSID);
  }

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
  Serial.println("ESP NOW Initialized");

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer Added");
  }


  // Initialize pins

  pinMode(PRESSURE_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(EC_PIN, INPUT);
  pinMode(HIGH_PERISTALTIC_PIN_1, OUTPUT);
  pinMode(HIGH_PERISTALTIC_PIN_2, OUTPUT);
  pinMode(PERISTALTIC_PIN_3, OUTPUT);





// numberOfDevices = sensors.getDeviceCount();
  
//   // locate devices on the bus
//   Serial.print("Locating devices...");
//   Serial.print("Found ");
//   Serial.print(numberOfDevices, DEC);
//   Serial.println(" devices.");

//   for(int i=0;i<numberOfDevices; i++) {
//     // Search the wire for address
//     if(sensors.getAddress(tempDeviceAddress, i)) {
//       Serial.print("Found device ");
//       Serial.print(i, DEC);
//       Serial.print(" with address: ");
//       for (int j=0; j<8; j++){
//         printHex(tempDeviceAddress[j]);
//       }
//       Serial.println();
//     } else {
//       Serial.print("Found ghost device at ");
//       Serial.print(i, DEC);
//       Serial.print(" but could not detect address. Check power and cabling");
//     }
//   }
  
//   Serial.print("Device 0 ");
//   if (sensors.isConnected(tempDeviceAddress)){
//     Serial.print("IS ");
//   }
//   else {
//     Serial.print("IS NOT ");
//   }
//   Serial.println("connected ");
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long F1StartMillis;
  unsigned long E1StartMillis;
  unsigned long F2StartMillis;

  // Maintain WiFi connection
  int32_t channel = getWiFiChannel(WIFI_SSID);
  channel = getWiFiChannel(WIFI_SSID);
  while (channel < 1) {
    delay(1000);
    Serial.println("WiFi Channel Not Found!");
    channel = getWiFiChannel(WIFI_SSID);
  }

  if(sampling){
    digitalWrite(HIGH_PERISTALTIC_PIN_2,HIGH);
    E1StartMillis = millis();
    Serial.print("Emptying Sampling Chamber");
    while((currentMillis - E1StartMillis)<EMPTY_DURATION){
      currentMillis = millis();
      Serial.print(".");
    }
    digitalWrite(HIGH_PERISTALTIC_PIN_2,LOW);

    digitalWrite(HIGH_PERISTALTIC_PIN_1,HIGH);
    F1StartMillis = millis();
    Serial.print("Filling Sampling Chamber");
    while((currentMillis - F1StartMillis)<FILL_DURATION){
      currentMillis = millis();
      Serial.print(".");
    }
    digitalWrite(HIGH_PERISTALTIC_PIN_1,LOW);

    sampling_sequence();

    digitalWrite(HIGH_PERISTALTIC_PIN_2,HIGH);
    E1StartMillis = millis();
    Serial.print("Emptying Sampling Chamber");
    while((currentMillis - E1StartMillis)<EMPTY_DURATION){
      currentMillis = millis();
      Serial.print(".");
    }
    digitalWrite(HIGH_PERISTALTIC_PIN_2,LOW);

    digitalWrite(PERISTALTIC_PIN_3,HIGH);
    F2StartMillis = millis();
    Serial.print("Filling Sampling Chamber again");
    while((currentMillis - F2StartMillis)<EMPTY_DURATION){
      currentMillis = millis();
      Serial.print(".");
    }
    digitalWrite(PERISTALTIC_PIN_3,LOW);

    Serial.println("Sampling Complete");
    sampling = false;
  }
  
  if(is_send_data){
    esp_err_t result = esp_now_send(0, (uint8_t*) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
      is_send_data = false;
    }
    else {
      Serial.println("Error sending the data");
    }
  }



  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet;
  esp_now_register_send_cb(OnDataSent);
  delay(1000);
  
}



