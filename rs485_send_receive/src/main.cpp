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

#define EN_1 12

// RS485 pins in use
#define RX_PIN 16    // Soft Serial Receive pin, connected to RO //  
#define TX_PIN 17    // Soft Serial Transmit pin, connected to DI // 
#define CTRL_PIN 26  // RS485 Direction control, connected to RE and DE // 


// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

//variables
int baud_rate = 9600;
byte command[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xc4, 0x0b};
byte address = 0x1;
int num_bytes = 7;

// standard commands
// CWT check slaveID {0xFF, 0x03, 0x07, 0xD0, 0x00, 0x01, 0x91, 0x59}
// CWT read {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08}
// rk520-01 read {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b}
// rk500-03 read {0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0xc4, 0x0b}

void request_reading();
void receive_reading(int num_bytes);

void setup() {

  Serial.begin(baud_rate);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);

  CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, address, baud_rate, protocol);  // Serial connection setup
  request_reading();
  receive_reading(num_bytes);



}

void loop() {

}

void request_reading() {

  command[0] = address;
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}


void receive_reading(int num_bytes){
  byte reading[num_bytes];
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
}

