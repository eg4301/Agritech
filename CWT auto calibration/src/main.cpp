#include <stdint.h>
#include <Arduino.h>

#include <sstream>
#include <iomanip>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>

#include "Sol16_RS485.h"
#include "SoftwareSerial.h"

/* Public defines ----------------------------------------------------------- */
#define DEEPSLEEPDURATION (24 * 60 * 60)  // Time interval between readings, in seconds (default 24 hours)
#define ADDRESS (0x01)                    // ! NEED TO CHANGE FOR EACH WATER METER

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

byte hexI[]={0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
byte readings[][8][10]={};
// readings[i][0] = address;
// readings[i][1] = humidity(%);
// readings[i][2] = temperature(C);
// readings[i][3] = conductivity(us/cm);
// readings[i][4] = PH;
// readings[i][5] = N(mg/kg);
// readings[i][6] = P(mg/kg);
// readings[i][7] = K(mg/kg);


void request_reading_CWT();
void request_reading_rika();
void request_reading_moisture();
void receive_reading(byte reading[],int num_bytes);
void storeReadings(byte readings[][8][10], byte reading[], int address, int time);



void setup() {

  Serial.begin(9600);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);


  // Take readings for all sensors, for 10 points of data with interval of 5 minutes
  for (int j = 0; j < 5; j++) {
    for(int i = 0; i < 5; i++) {
      // Read and send CWT data
      CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, hexI[i], baud_rate, protocol);  // Serial connection setup
      byte reading[] {};
      request_reading_CWT(hexI[i]);
      receive_reading(reading,19);
      storeReadings(readings, reading, i,j);

      delay(1000);
    }

    // Read and send Rika all in one data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x06, baud_rate, protocol);  // Serial connection setup
    byte reading[] {};
    request_reading_rika();
    receive_reading(reading,13);
    storeReadings(readings, reading, 6,j);

    delay(1000);

    // // Read and send Rika moisture data
    // CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x07, baud_rate, protocol);  // Serial connection setup
    // byte reading[] {};
    // request_reading_moisture();
    // receive_reading(reading,7);
    // storeReadings(readings, reading, 7, j);

    // delay(1000);
    delay(300000);
  }

  // Calculate formula for each sensor for values of moisture and EC
  

}

void request_reading_CWT(byte address) {
  byte command[8] ={address, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  CWT_Sensor.request_reading(command, 8);
}

void request_reading_rika() {
  byte command[8] = {0x06, 0x03, 0x00, 0x00, 0x00, 0x04, 0x45, 0xBE};
  CWT_Sensor.request_reading(command, 8);
}

void request_reading_moisture() {
  byte command[8] = {0x07, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x6c};
  CWT_Sensor.request_reading(command, 8);
}

void receive_reading(byte reading[], int num_bytes) {
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
}

void storeReadings(byte readings[][8][10], byte reading[], int address, int time) {
  readings[address][0][time] = reading[0];
  readings[address][1][time] = (reading[3] << 8 | reading[4])/10;
  readings[address][2][time] = (reading[5] << 8 | reading[6])/10;
  readings[address][3][time] = (reading[7] << 8 | reading[8])/10;
  readings[address][4][time] = (reading[9] << 8 | reading[10])/10;
  readings[address][5][time] = (reading[11] << 8 | reading[12]);
  readings[address][6][time] = (reading[13] << 8 | reading[14]);
  readings[address][7][time] = (reading[15] << 8 | reading[16]); 
}
