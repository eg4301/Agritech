#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>

#include "Sol16_RS485.h"

/* Public defines ----------------------------------------------------------- */
#define DEEPSLEEPDURATION (24 * 60 * 60)  // Time interval between readings, in seconds (default 24 hours)
#define ADDRESS (0x01)                    // ! NEED TO CHANGE FOR EACH WATER METER

#define EN_1 12

// RS485 pins in use
#define RX_PIN 16    // Soft Serial Receive pin, connected to RO // ! PINOUT TBC ONCE PCB ARRIVES
#define TX_PIN 17    // Soft Serial Transmit pin, connected to DI // ! PINOUT TBC ONCE PCB ARRIVES
#define CTRL_PIN 26  // RS485 Direction control, connected to RE and DE // ! PINOUT TBC ONCE PCB ARRIVES

#define baud_rate 4800

// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

byte reading[19];

/* Private Constants -------------------------------------------------------- */
uint8_t broadcastAddress[] = { 0x16, 0x16, 0x16, 0x16, 0x16, 0x16 };  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  byte reading[19];
} struct_sensor_reading;

struct_sensor_reading myData;

esp_now_peer_info_t peerInfo;

void request_reading();
byte receive_reading();
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

void setup() {

  Serial.begin(115200);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);

  CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, ADDRESS, baud_rate, protocol);  // Serial connection setup

  // Sensor specific data
  request_reading();
  receive_reading();

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  memcpy(myData.reading, reading, 19);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}

void request_reading() {
  byte command[8];

  command[0] = ADDRESS;  // Set the address of the water meter based on last 2 digits
  command[1] = 0x03;     // Set the function code for reading data register, given in datasheet
  command[2] = 0x00;     // Starting address, high byte
  command[3] = 0x00;     // Starting address, low byte
  command[4] = 0x00;     // Number of registers, high byte
  command[5] = 0x07;     // Number of registers, low byte
  command[6] = 0x04;
  command[7] = 0x08;
  CWT_Sensor.request_reading(command, 8);
}

byte receive_reading() {
  int num_bytes = 19;
  byte reading[19];
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
  return reading[19];
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}