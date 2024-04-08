#include <stdio.h>
#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>

// CO2, Temperature and Humidity sensor library
#include "SCD30.h"

// O2 Sensor Library
#include "DFRobot_OxygenSensor.h"
#define Oxygen_IICAddress ADDRESS_3
#define COLLECT_NUMBER  10             // collect number, the collection range is 1-100.
DFRobot_OxygenSensor oxygen;



#define flatOff_temp 0            // Flat deviation compensate
#define scaleOff_temp 1           // Scale deviation compensate
#define flatOff_hum 0            // Flat deviation compensate
#define scaleOff_hum 1           // Scale deviation compensate

float atmTemp = 25.0;
float atmHum = 50.0;
float result[3] = {0};
float CO2;
float err;
float O2;
// String MACaddr;
// char charMAC[17] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7};


// put function declarations here:
uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x96, 0xF2, 0x14};  // ! REPLACE WITH YOUR RECEIVER MAC Address

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

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// put function definitions here:
void getCO2HumTemp(){
  // Checks if the CO2 Sensor is discoverable on the I2C bus
  if (scd30.isAvailable()){
    // Requests from CO2 Sensor (slave)
    scd30.getCarbonDioxideConcentration(result);
    CO2 = result[0];
    Serial.println(CO2);
    atmTemp = result[1];
    Serial.println(atmTemp);
    atmHum = result[2];
    Serial.println(atmHum);
  }
}


void getO2(){
  O2 = oxygen.getOxygenData(COLLECT_NUMBER);
  Serial.println(O2);
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  // MACaddr = WiFi.macAddress();
  // strcpy(charMAC,MACaddr.c_str());
  // Serial.println("Below is charMAC");
  // Serial.println(charMAC);
  // Serial.println("Above is charMAC");
  myData.MAC = 2;

  // Start SCD30 CO2 Sensor
  scd30.initialize();
  Serial.println("SCD30 Sensor started");

  // Start O2 Sensor
  while(!oxygen.begin(Oxygen_IICAddress)){
    Serial.println("I2c device number error !");
    delay(1000);
  }
  Serial.println("I2c connect success !");

  // Start WiFi and enable LR
  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer Added");
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  getCO2HumTemp();
  getO2();
  myData.atmtemp = atmTemp;
  myData.hum = atmHum;
  myData.CO2 = CO2;
  myData.Oxy = O2;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  delay(600000);
}

