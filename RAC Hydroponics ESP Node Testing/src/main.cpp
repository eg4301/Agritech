#include <stdint.h>
#include <Arduino.h>



#define PUMP_PIN_1 4
#define PUMP_PIN_2 5
#define PUMP_PIN_3 6
#define PUMP_PIN_4 7
#define PUMP_PIN_5 8
#define PUMP_PIN_6 3
#define PUMP_PIN_7 9
#define VALVE_PIN 10

#define PUMP_DURATION 1000
#define VALVE_DURATION 1000

char pumplist[][25] = 
{
  "PUMP_PIN_1",
  "PUMP_PIN_2",
  "PUMP_PIN_3",
  "PUMP_PIN_4",
  "PUMP_PIN_5",
  "PUMP_PIN_6",
  "PUMP_PIN_7",
};

void sampling_seq() {
  for (int i = 0; i < (sizeof(pumplist)) / sizeof(pumplist[0]); i++) {
    digitalWrite(int(pumplist[i]), HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(int(pumplist[i]), LOW); 

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

  sampling_seq();
}

void loop() {
  
  
  
}
