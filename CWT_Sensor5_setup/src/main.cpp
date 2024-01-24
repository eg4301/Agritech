#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>

#include "Sol16_RS485.h"
#include "SoftwareSerial.h"


/* Public defines ----------------------------------------------------------- */
#define DEEPSLEEPDURATION (24 * 60 * 60)  // Time interval between readings, in seconds (default 24 hours)
#define ADDRESS (0x01)                    // ! NEED TO CHANGE FOR EACH sensor type

#define EN_1 12

// RS485 pins in use
#define RX_PIN 16    // Soft Serial Receive pin, connected to RO //  
#define TX_PIN 17    // Soft Serial Transmit pin, connected to DI // 
#define CTRL_PIN 26  // RS485 Direction control, connected to RE and DE // 

#define baud_rate 4800

// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

//List of slave IDs
byte slaveID[] = {0x01, 0x02, 0x03, 0x04, 0x05};

void request_reading(byte SLAVE_ID);
byte receive_reading();
void check_slaveID();
byte receive_reading_ID();
void add_crc(byte reading[], int start, int size);

void setup() {

  Serial.begin(4800);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);

  CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, ADDRESS, baud_rate, protocol);  // Serial connection setup

  // // Check byte[0] for address
  // check_slaveID();
  // Serial.println(receive_reading_ID());

  // Sensor specific data
  for(int i=0; i <= 4; i++ ){
    Serial.println("Connect your sensor to board, then type any number into the input and press Enter to continue...");

    Serial.read();
    while (!Serial.available()) {
      delay(1000);
    }
    while (Serial.available()) {
      Serial.read();
      delay(1000);
    }

    
    request_reading(slaveID[i]);

    Serial.println(receive_reading());
  }
  Serial.println("5 sensors initialized");


}

void loop() {

}

void request_reading(byte SLAVE_ID) {
  byte command[8];

  command[0] = 0x01;  // Set the address of the water meter based on last 2 digits
  command[1] = 0x06;     // Set the function code for reading data register, given in datasheet
  command[2] = 0x07;     // Starting address, high byte
  command[3] = 0xD0;     // Starting address, low byte
  command[4] = 0x00;     // Number of registers, high byte
  command[5] = SLAVE_ID;     // Number of registers, low byte

  CWT_Sensor.add_crc(command, 0, 6);

  CWT_Sensor.request_reading(command, 8);
}

byte receive_reading() {
  int num_bytes = 8;
  byte reading[8];
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
  return reading[8];
}

void check_slaveID() {
  byte command[8];

  command[0] = 0xFF;  // Set the address of the water meter based on last 2 digits
  command[1] = 0x03;     // Set the function code for reading data register, given in datasheet
  command[2] = 0x07;     // Starting address, high byte
  command[3] = 0xD0;     // Starting address, low byte
  command[4] = 0x00;     // Number of registers, high byte
  command[5] = 0x01;     // Number of registers, low byte
  command[6] = 0x91;
  command[7] = 0x59;
  CWT_Sensor.request_reading(command, 8);

}

byte receive_reading_ID(){
  int num_bytes = 7;
  byte reading[7];
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
  return reading[0];
}

