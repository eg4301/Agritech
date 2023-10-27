#include <stdint.h>
#include <Arduino.h>


// When changin pins, please also change the pin numbers manually in pumplist!!!
#define PUMP_PIN_1 9
#define PUMP_PIN_2 10
#define PUMP_PIN_3 11
#define PUMP_PIN_4 12
#define PUMP_PIN_5 13
#define PUMP_PIN_6 14
#define PUMP_PIN_7 15
#define VALVE_PIN 16

// Define length of time pumps and valves are open
#define PUMP_DURATION 1000
#define VALVE_DURATION 1000

// Chang here too!!!
int pumplist[] = 
{
  9,
  10,
  11,
  12,
  13,
  14
};

void sampling_seq() {
  for (int i = 0; i < (sizeof(pumplist)) / sizeof(pumplist[0]); i++) {

    digitalWrite(pumplist[i], HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(pumplist[i], LOW); 

    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW);

    digitalWrite(int(PUMP_PIN_7), HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(int(PUMP_PIN_7), LOW); 

    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW);

    Serial.println(pumplist[i]);
  }
}


void setup() {
  Serial.begin(115200);


  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(PUMP_PIN_4, OUTPUT);
  pinMode(PUMP_PIN_5, OUTPUT);
  pinMode(PUMP_PIN_6, OUTPUT);
  pinMode(PUMP_PIN_7, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

  // sampling_seq();

}

void loop() {
  
  
  
}
