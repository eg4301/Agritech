#include <stdint.h>
#include <Arduino.h>

#include <sstream>
#include <iomanip>

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
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

// Soil sensor constants
#define totSensors 6  // Total number of sensors
#define numReadingTypes 5 // Number of readings types to be taken
#define numReadings 5 // Number of readings to be taken


Config protocol = SWSERIAL_8N1;

Preferences preferences;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

byte hexI[5] {0x01, 0x02, 0x03, 0x04, 0x05};
// byte hexI[] {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int numCWT = sizeof(hexI);

byte readings[totSensors][numReadingTypes][numReadings] {};
// readings[i][0] = address;
// readings[i][1] = humidity(%);
// readings[i][2] = temperature(C);
// readings[i][3] = conductivity(us/cm);
// readings[i][4] = PH;
// The below will not be used in the current code, but can be implemented in the future
//// readings[i][5] = N(mg/kg);
//// readings[i][6] = P(mg/kg);
//// readings[i][7] = K(mg/kg);
byte quickreadings[totSensors][numReadingTypes] {};
//instant version of readings

byte sensorTrends[totSensors][numReadingTypes][2] {};
// sensorTrends[i][j][0] = slope;
// sensorTrends[i][j][1] = intercept;

byte sensorTransform[totSensors][numReadingTypes][3] {};
// sensorTrends[i][j][0] = c1;
// sensorTrends[i][j][1] = m2/m1;
// sensorTrends[i][j][2] = c2;

// Function init
void request_reading_CWT(byte address);
void request_reading_rika();
void request_reading_moisture();
void receive_reading(byte reading[totSensors],int num_bytes);
void storeReadings(byte readings[totSensors][numReadingTypes][numReadings], byte reading[19], int address, int time, int sensType);
void printReadings(byte readings[totSensors][numReadingTypes][numReadings]);

void calcTrend(byte readings[totSensors][numReadingTypes][numReadings], byte sensorTrends[totSensors][numReadingTypes][2]);
void LSRL(byte readings[numReadings], byte sensorTrends[2]);
int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val);
void printQuickReadings(byte quickreadings[totSensors][numReadingTypes]);
void storeReadingsCorrected(byte quickreadings[totSensors][numReadingTypes], byte reading[], int sensNum, int time, int sensType);
void storeTransform(byte sensorTrends[totSensors][numReadingTypes][2]);

void setup() {

  Serial.begin(9600);
  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);


  // Take readings for all sensors, for 5 points of data with interval of 5 minutes
  // Stores all readings into readings[(sensor)][(parameter)][(reading number)]
  for (int j = 0; j < 5; j++) {
    
    byte reading[19] {};

    for(int i = 0; i < numCWT; i++) {
      // Read and send CWT data
      CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, hexI[i], baud_rate, protocol);  // Serial connection setup

      request_reading_CWT(hexI[i]);
      receive_reading(reading,19);
      storeReadings(readings, reading, i,j, 1);

      delay(1000);

    }

    // Read and send Rika all in one data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x06, baud_rate, protocol);  // Serial connection setup

    request_reading_rika();
    receive_reading(reading,13);

    storeReadings(readings, reading, numCWT, j, 2);

    delay(1000);

    printReadings(readings);
    delay(20000);
  }

  // Calculate equations using linear regression for each sensor for all parameters
  // Stores it into sensorTrends[(sensors)][(parameters)][(slope,intercept)]
  calcTrend(readings, sensorTrends);
  storeTransform(sensorTrends);

  // 
}

void loop() {
  // put your main code here, to run repeatedly:
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
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}

void receive_reading(byte reading[], int num_bytes) {
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
}

void storeReadings(byte readings[totSensors][numReadingTypes][numReadings], byte reading[], int sensNum, int time, int sensType) {
  switch (sensType)
  {
  case 1:
    readings[sensNum][0][time] = reading[0];
    readings[sensNum][1][time] = (reading[3] << 8 | reading[4])/10;
    readings[sensNum][2][time] = (reading[5] << 8 | reading[6])/10;
    readings[sensNum][3][time] = (reading[7] << 8 | reading[8]);
    readings[sensNum][4][time] = (reading[9] << 8 | reading[10])/10;
    break;
  
  case 2:
    readings[sensNum][0][time] = reading[0];
    readings[sensNum][1][time] = (reading[5] << 8 | reading[6])/10;
    readings[sensNum][2][time] = (reading[3] << 8 | reading[4])/10;
    readings[sensNum][3][time] = (reading[7] << 8 | reading[8]);
    readings[sensNum][4][time] = (reading[9] << 8 | reading[10])/100;
    break;
  }

}


void printReadings(byte readings[totSensors][numReadingTypes][numReadings]) {
  for (int sensor = 0; sensor < totSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor+1);
    Serial.println(" Readings:");

    for (int readingType = 0; readingType < numReadingTypes; readingType++) {
      switch(readingType) {
        case 0: Serial.print("Address: "); break;
        case 1: Serial.print("Humidity: "); break;
        case 2: Serial.print("Temperature: "); break;
        case 3: Serial.print("Conductivity: "); break;
        case 4: Serial.print("PH: "); break;
      }
      for (int time = 0; time < numReadings; time++) {
        Serial.print(readings[sensor][readingType][time]); 
        Serial.print(" ");
      }
      Serial.println();
    }
    Serial.println("---------------------------");
  }
}

void LSRL(byte readings[numReadings], byte sensorTrends[2]){ //Least square regression
  float sumX = 0;
  float sumY = 0;
  float sumXY = 0;
  float sumX2 = 0;
  float n = numReadings;

  for (int i = 0; i < numReadings; i++) {
    sumX += i;
    sumY += readings[i];
    sumXY += i*readings[i];
    sumX2 += i*i;
  }

  float slope = (n*sumXY - sumX*sumY)/(n*sumX2 - sumX*sumX);
  float intercept = (sumY - slope*sumX)/n;

  sensorTrends[0] = slope;
  sensorTrends[1] = intercept;
}

void calcTrend(byte readings[totSensors][numReadingTypes][numReadings], byte sensorTrends[totSensors][numReadingTypes][2]) {
  for (int sensor = 0; sensor < totSensors; sensor++) {

    Serial.print("Sensor ");
    Serial.print(sensor+1);
    Serial.println(" equations:");

    for (int readingType = 1; readingType < numReadingTypes; readingType++) {
      LSRL(readings[sensor][readingType], sensorTrends[sensor][readingType]);
      
      Serial.print("y = ");
      Serial.print(sensorTrends[sensor][readingType][0]);
      Serial.print("x + ");
      Serial.println(sensorTrends[sensor][readingType][1]);  
      
    }
    Serial.println("---------------------------");
  }
}

int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val) {
  // y2 = (y1-c1) x (m2/m1) + c2
  return (val - sensorTransform[sensNum][readingType][0]) * sensorTransform[sensNum][readingType][1] + sensorTransform[sensNum][readingType][2];
}

void storeTransform(byte sensorTrends[totSensors][numReadingTypes][2]) {
  preferences.begin("Transform", false);

  for (int readingType = 1; readingType < numReadingTypes; readingType++) {
      
      float c2 = sensorTrends[totSensors][readingType][0]; 
      float m2 = sensorTrends[totSensors][readingType][1]; 

      for (int sensor = 0; sensor < totSensors-1; sensor++)  {
      // Construct the key as "sensori_j_0" and "sensor_i_j_1" for slope and intercept respectively
      String keyC1 = "sensor" + String(sensor) + "_" + String(readingType) + "0";
      String keym2_m1 = "sensor" + String(sensor) + "_" + String(readingType) + "1";
      String keyC2 = "sensor" + String(sensor) + "_" + String(readingType) + "2";

      // Store the slope and intercept values
      preferences.putFloat(keyC1.c_str(), sensorTrends[sensor][readingType][0]);
      preferences.putFloat(keym2_m1.c_str(), m2/sensorTrends[sensor][readingType][1]);
      preferences.putFloat(keyC2.c_str(), c2);
      Serial.println("trans stored");
    }
  }

  preferences.end();
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
      sensorTransform[sensor][readingType][0] = preferences.getFloat(keyC1.c_str(), sensorTrends[sensor][readingType][0]);
      sensorTransform[sensor][readingType][1] = preferences.getFloat(keym2_m1.c_str(), sensorTrends[sensor][readingType][1]);
      sensorTransform[sensor][readingType][2] = preferences.getFloat(keyC2.c_str(), sensorTrends[sensor][readingType][2]);
    }
  }

  preferences.end();
}



void printQuickReadings(byte quickreadings[totSensors][numReadingTypes]) {
  for (int sensor = 0; sensor < totSensors; sensor++) {
    Serial.print("Sensor ");
    Serial.print(sensor+1);
    Serial.println("Quick Reading:");

    for (int readingType = 0; readingType < numReadingTypes; readingType++) {
      switch(readingType) {
        case 0: Serial.print("Address: "); break;
        case 1: Serial.print("Humidity: "); break;
        case 2: Serial.print("Temperature: "); break;
        case 3: Serial.print("Conductivity: "); break;
        case 4: Serial.print("PH: "); break;
      }
      Serial.print(transform(sensorTransform, sensor, readingType, quickreadings[sensor][readingType]));
      Serial.println();
    }
    Serial.println("---------------------------");
  }
}


void storeReadingsCorrected(byte quickreadings[totSensors][numReadingTypes], byte reading[], int sensNum, int time, int sensType) {
  switch (sensType)
  {
  case 1:
    readings[sensNum][0][time] = reading[0];
    readings[sensNum][1][time] = transform(sensorTransform, sensNum, 1, (reading[3] << 8 | reading[4])/10);
    readings[sensNum][2][time] = transform(sensorTransform, sensNum, 2, (reading[5] << 8 | reading[6])/10);
    readings[sensNum][3][time] = transform(sensorTransform, sensNum, 3, (reading[7] << 8 | reading[8]));
    readings[sensNum][4][time] = transform(sensorTransform, sensNum, 4, ((reading[9] << 8 | reading[10])/10));
    break;
  
  case 2:
    readings[sensNum][0][time] = reading[0];
    readings[sensNum][1][time] = transform(sensorTransform, sensNum, 1, (reading[5] << 8 | reading[6])/10);
    readings[sensNum][2][time] = transform(sensorTransform, sensNum, 2, (reading[3] << 8 | reading[4])/10);
    readings[sensNum][3][time] = transform(sensorTransform, sensNum, 3, (reading[7] << 8 | reading[8]));
    readings[sensNum][4][time] = transform(sensorTransform, sensNum, 4, (reading[9] << 8 | reading[10])/100);
    break;
  }

}