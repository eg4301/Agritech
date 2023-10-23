#include <stdint.h>
#include <Arduino.h>

#include <EEPROM.h>
#include "DFRobot_EC.h"

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>

#define LED_PIN     25

#define PUMP_PIN_1 4
#define PUMP_PIN_2 5
#define PUMP_PIN_3 6
#define PUMP_PIN_4 7
#define PUMP_PIN_5 18
#define PUMP_PIN_6 3
#define PUMP_PIN_7 9
#define VALVE_PIN 10

#define PUMP_DURATION 1000
#define VALVE_DURATION 1000

char pumplist[] = 
{
  "PUMP_PIN_1",
  "PUMP_PIN_2",
  "PUMP_PIN_3",
  "PUMP_PIN_4",
  "PUMP_PIN_5",
  "PUMP_PIN_6",
  "PUMP_PIN_7",
};

void setup() {
  Serial.begin(115200);
  
  /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
  led_strip_set_pixel(led_strip, 0, 16, 16, 16);
  /* Refresh the strip to send data */
  led_strip_refresh(led_strip);

  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(PUMP_PIN_4, OUTPUT);
  pinMode(PUMP_PIN_5, OUTPUT);
  pinMode(PUMP_PIN_6, OUTPUT);
  pinMode(PUMP_PIN_7, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  sampling_seq();
}

void loop() {
  

  
}

void sampling_seq() {
  for (int i = 0; i < (sizeof(pumplist)) / sizeof(pumplist[0]); i++) {
    digitalWrite(pumplist[i], HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(pumplist[i], LOW); 
    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW); 
  }
}