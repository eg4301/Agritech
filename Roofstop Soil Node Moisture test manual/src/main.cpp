#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>

#include "Sol16_RS485.h"
#include "SoftwareSerial.h"

#define EN_1 12

// RS485 pins in use
#define RX_PIN 16    // Soft Serial Receive pin, connected to RO // ! PINOUT TBC ONCE PCB ARRIVES
#define TX_PIN 17    // Soft Serial Transmit pin, connected to DI // ! PINOUT TBC ONCE PCB ARRIVES
#define CTRL_PIN 26  // RS485 Direction control, connected to RE and DE // ! PINOUT TBC ONCE PCB ARRIVES

#define baud_rate 9600

// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

byte HexI[] = {0x01, 0x02, 0x03, 0x04, 0x05};



void request_reading_CWT(byte address);
void request_reading_rika();
void request_reading_moisture();
void receive_reading(byte reading[],int num_bytes);


void setup() {

  Serial.begin(9600);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(2000);
  Serial.println("buffer");
  Serial.println("buffer");
  Serial.println("buffer");
  Serial.println("CLEARSHEET");
  Serial.println("Address,Minutes,Sensor,Temperature,Moisture,EC,pH,N,P,K");
}

void loop() {
  
  for (int i = 0; i < 1; i++) {
    byte reading[19] {};
    for (int j = 0; j < sizeof(HexI); j++) {
          // Read and send CWT data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, HexI[j], baud_rate, protocol);  // Serial connection setup
    request_reading_CWT(HexI[j]);
    receive_reading(reading,19);
    Serial.print("DATA,");
    Serial.print(millis()/60000);
    Serial.print("," + (String)(reading[0]) + ",");
    Serial.print((String)((reading[5] << 8 | reading[6])/10)+",");
    Serial.print((String)((reading[3] << 8 | reading[4])/10)+",");
    Serial.print((String)(reading[7] << 8 | reading[8])+",");
    Serial.print((String)((reading[9] << 8 | reading[10])/10)+",");
    Serial.print((String)(reading[11] << 8 | reading[12])+",");
    Serial.print((String)(reading[13] << 8 | reading[14])+",");
    Serial.println((String)((reading[15]<< 8 | reading[16])/10));

    delay(1000);
    }


    // Read and send Rika all in one data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x06, baud_rate, protocol);  // Serial connection setup
    request_reading_rika();
    receive_reading(reading,13);
    Serial.print("DATA,");
    Serial.print(millis()/60000);
    Serial.print("," + (String)(reading[0]) + ",");
    Serial.print((String)((reading[3] << 8 | reading[4])/10)+",");
    Serial.print((String)((reading[5] << 8 | reading[6])/10)+",");
    Serial.print((String)(reading[7] << 8 | reading[8])+",");
    Serial.println((String)((reading[9] << 8 | reading[10])/100));

    delay(1000);

    // // Read and send Rika moisture data
    // CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x07, baud_rate, protocol);  // Serial connection setup
    // request_reading_moisture();
    // receive_reading(reading,7);
    // Serial.print("DATA,");
    // Serial.print(millis()/1000);
    // Serial.print(",Rika Moisture,Null,");
    // Serial.println((String)((reading[3] << 8 | reading[4])/10));

    // delay(10000);
  }

  delay(600000);
}

void request_reading_CWT(byte address) {
  byte command[8] ={address, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}

void request_reading_rika() {
  byte command[8] = {0x06, 0x03, 0x00, 0x00, 0x00, 0x04, 0x45, 0xBE};
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}

void request_reading_moisture() {
  byte command[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x6c};
  CWT_Sensor.request_reading(command, 8);
}

void receive_reading(byte reading[], int num_bytes) {
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
}

