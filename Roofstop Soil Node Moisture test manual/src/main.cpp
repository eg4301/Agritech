#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>

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

// Soil sensor constants
#define totSensors 2  // Total number of sensors
#define numReadingTypes 5 // Number of readings types to be taken
#define numReadings 5 // Number of readings to be taken

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Preferences preferences;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

byte HexI[] = {0x01, 0x02, 0x03, 0x04, 0x05};

byte sensorTransform[totSensors][numReadingTypes][3] {};
// sensorTrends[i][j][0] = c1;
// sensorTrends[i][j][1] = m2/m1;
// sensorTrends[i][j][2] = c2;
// readings[i][0] = address;
// readings[i][1] = humidity(%);
// readings[i][2] = temperature(C);
// readings[i][3] = conductivity(us/cm);
// readings[i][4] = PH;



void request_reading_CWT(byte address);
void request_reading_rika();
void request_reading_moisture();
void receive_reading(byte reading[],int num_bytes);

void retrieveTransform(byte sensorTransform[totSensors][numReadingTypes][3]);
int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val);

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
  
  for (int i = 0; i < 6; i++) {
    byte reading[19] {};
    for (int j = 0; j < sizeof(HexI); j++) {
    // Read and send CWT data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, HexI[j], baud_rate, protocol);  // Serial connection setup
    request_reading_CWT(HexI[j]);
    receive_reading(reading,19);

    // int x1 = transform(sensorTransform, j, 2, (reading[5] << 8 | reading[6])/10);
    // int x2 = transform(sensorTransform, j, 1, (reading[3] << 8 | reading[4])/10);
    // int x3 = transform(sensorTransform, j, 3, reading[7] << 8 | reading[8]);
    // int x4 = transform(sensorTransform, j, 4, (reading[9] << 8 | reading[10])/10);
    int x1 = (reading[5] << 8 | reading[6])/10;
    int x2 = (reading[3] << 8 | reading[4])/10;
    int x3 = reading[7] << 8 | reading[8];
    int x4 = (reading[9] << 8 | reading[10])/10;

    // print data for PLX DAQ
    Serial.print("DATA,");
    Serial.print(millis()/60000);
    Serial.print("," + (String)(reading[0]) + ",");
    Serial.print((String)(x1)+",");
    Serial.print((String)(x2)+",");
    Serial.print((String)(x3)+",");
    Serial.print((String)(x4)+",");
    // Serial.print((String)(reading[11] << 8 | reading[12])+",");
    // Serial.print((String)(reading[13] << 8 | reading[14])+",");
    // Serial.println((String)((reading[15]<< 8 | reading[16])/10));

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

  delay(6000);
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

void retrieveTransform(byte sensorTransform[totSensors][numReadingTypes][3]) {
  preferences.begin("Transform", true);

  for (int sensor = 0; sensor < totSensors; sensor++) {
      for (int readingType = 1; readingType < numReadingTypes; readingType++) {
      // Construct the key as "sensori_j_0" and "sensor_i_j_1" for slope and intercept respectively
      String keyC1 = "sensor" + String(sensor) + "_" + String(readingType) + "0";
      String keym2_m1 = "sensor" + String(sensor) + "_" + String(readingType) + "1";
      String keyC2 = "sensor" + String(sensor) + "_" + String(readingType) + "2";

      // Store the slope and intercept values
      sensorTransform[sensor][readingType][0] = preferences.getFloat(keyC1.c_str(), 1);
      sensorTransform[sensor][readingType][1] = preferences.getFloat(keym2_m1.c_str(), 1);
      sensorTransform[sensor][readingType][2] = preferences.getFloat(keyC2.c_str(), 1);
    }
  }

  preferences.end();
}

int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val) {
  // y2 = (y1-c1) x (m2/m1) + c2
  return (val - sensorTransform[sensNum][readingType][0]) * sensorTransform[sensNum][readingType][1] + sensorTransform[sensNum][readingType][2];
}
