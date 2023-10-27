#include <stdint.h>
#include <Arduino.h>



#define PUMP_PIN_1 9
#define PUMP_PIN_2 10
#define PUMP_PIN_3 11
#define PUMP_PIN_4 12
#define PUMP_PIN_5 13
#define PUMP_PIN_6 14
#define PUMP_PIN_7 15
#define VALVE_PIN 16

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
};

void sampling_seq() {
  for (int i = 0; i < (sizeof(pumplist)) / sizeof(pumplist[0]); i++) {
    
    char* x = pumplist[i];

    digitalWrite(int(x), HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(int(x), LOW); 

    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW);

    digitalWrite(int(PUMP_PIN_7), HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(int(PUMP_PIN_7), LOW); 

    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW);

    Serial.println(x);
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

  // digitalWrite(int(PUMP_PIN_1), HIGH);
  // digitalWrite(int(PUMP_PIN_2), HIGH);
  // digitalWrite(int(PUMP_PIN_3), HIGH);
  // digitalWrite(int(PUMP_PIN_4), HIGH);
  // digitalWrite(int(PUMP_PIN_5), HIGH);
  // digitalWrite(int(PUMP_PIN_6), HIGH);
  // digitalWrite(int(PUMP_PIN_7), HIGH);
  // digitalWrite(int(VALVE_PIN), HIGH);

}

void loop() {
  
  
  
}
