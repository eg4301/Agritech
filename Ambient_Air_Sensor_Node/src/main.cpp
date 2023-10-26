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
#include "DFRobot_MultiGasSensor.h"
#define O2_I2C_ADDRESS 0x74




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
String MACaddr;
char* charMAC = new char[17];

DFRobot_GAS_I2C gas(&Wire, O2_I2C_ADDRESS);

// put function declarations here:
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x61, 0x8F, 0x58};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  char* MAC = new char[17];
  float temp = 0;
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
  if (scd30.isAvailable()){
    scd30.getCarbonDioxideConcentration(result);
    CO2 = result[0];
    atmTemp = result[1];
    atmHum = result[2];
  }
}

void getO2(){
  O2 = gas.readGasConcentrationPPM();
}

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  MACaddr = WiFi.macAddress();
  strcpy(charMAC,MACaddr.c_str());
  myData.MAC = charMAC;

  // Start SCD30 CO2 Sensor
  scd30.initialize();

  // Start SEN0469 O2 Sensor
  while(!gas.begin())
  {
    Serial.println("NO Devices!");
    delay(1000);
  }
  Serial.println("The device is connected successfully!");
  while(!gas.changeAcquireMode(gas.PASSIVITY)){
    delay(1000);
  }
  Serial.println("Acquire mode changed to passive.");

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
  myData.temp = atmTemp;
  myData.hum = atmHum;
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

  delay(30000);
}

