#include <Arduino.h>
#include <stdint.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>

// put function declarations here:
float A,B,C;

String MACaddr;
uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0xFE, 0x85, 0xBC};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  String MAC;
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



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  MACaddr = WiFi.macAddress();
  myData.MAC = MACaddr;
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
  A = random(10);
  Serial.println(A);
  B = random(10);
  Serial.println(B);
  C = random(10);
  Serial.println(C);

  myData.temp = A;
  myData.ECVal = B;
  myData.pHVal = C;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);

  delay(20000);
}

// put function definitions here:
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}