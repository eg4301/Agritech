#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>

// Temperature and Humidity sensor library
#include <DFRobot_SHT3x.h>

#define flatOff_temp 0            // Flat deviation compensate
#define scaleOff_temp 1           // Scale deviation compensate

float atmTemp = 25.0;

#define flatOff_hum 0            // Flat deviation compensate
#define scaleOff_hum 1           // Scale deviation compensate

float atmHum = 50.0;

// CO2 Sensor Library

// O2 Sensor Library

// put function declarations here:
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x61, 0x8F, 0x58};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  int MAC;
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Initialize SHT31-F
  while (sht3x.begin() != 0) {
    Serial.println("Failed to Initialize the chip, please confirm the wire connection");
    delay(1000);
  }

  /**
   * softReset Send command resets via IIC, enter the chip's default mode single-measure mode, 
   * turn off the heater, and clear the alert of the ALERT pin.
   * @return Read the register status to determine whether the command was executed successfully, 
   * and return true indicates success.
   */
   if(!sht3x.softReset()){
     Serial.println("Failed to Initialize the chip....");
   }

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
  atmTempRead();
  Serial.println(atmTemp);
  atmHumRead();
  Serial.println(atmHum);

  myData.temp = atmTemp;
  myData.hum = atmHum;

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

// put function definitions here:
void atmTempRead() {
  atmTemp = sht3x.getTemperatureC();
  atmTemp = scaleOff_temp * atmTemp + flatOff_temp;
}

void atmHumRead() {
  atmHum = sht3x.getHumidityRH();
  atmHum = scaleOff_hum * atmHum + flatOff_hum;
}