#include <stdint.h>
#include <Arduino.h>

#include <DFRobot_EC.h>

#include <Sol16_RS485.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include "driver/gpio.h"

#include "OneWire.h"
#include "DallasTemperature.h"

#define baud_rate 9600

// RS485 pins in use
#define RX_PIN 12    // Soft Serial Receive pin, connected to RO //  
#define TX_PIN 14    // Soft Serial Transmit pin, connected to DI // 
#define CTRL_PIN 13  // RS485 Direction control, connected to RE and DE // 


// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Norika water meter constants
#define RETURN_ADDRESS_IDX 0
#define RETURN_FUNCTIONCODE_IDX 1

Config protocol = SWSERIAL_8N1;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

// Declarations for Actuation (pumps + valve)
#define HIGH_PERISTALTIC_PIN_1 17 //Relay 2
#define HIGH_PERISTALTIC_PIN_2 18  //Relay 1 (pumps water opposite direction)
#define PERISTALTIC_PIN_1 7 //Relay 5
#define PERISTALTIC_PIN_2 15 //Relay 4
#define PERISTALTIC_PIN_3 16 //Relay 3
#define RECIRCULATING_PUMP 6 //Relay 6
#define IRRIGATION_PUMP 5 //Relay 7
#define WATER_VALVE 4  //Relay 8


// Define length of time pumps and valves are open

#define PUMP_DURATION 120000 //time used to pump clean water in ms
#define HIGH_PUMP_DURATION 6000 //time used to pump sample in ms
#define CLEAR_DURATION 20000 //time used to clear sample in ms
#define MIXING_DURATION 60000 //time used to mix sample in ms
#define IRRIGATION_DURATION 120000 //time used to pump sample in ms


// Declarations for pH Sensor:
#define PH_PIN 9             // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 15.82       // Flat deviation compensate
#define scaleOff_ph -5.66       // Scale deviation compensate
float avgRead_ph;             //Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value


// Declarations for EC Sensor:
#define EC_PIN 10

  // Using DFRobot library
// #define flatOff_ec 0.41                     // Flat deviation compensate
// #define scaleOff_ec 1.07                    // Scale deviation compensate
// float voltageRead,ecValue,temperature = 22;
// DFRobot_EC ec;

  // Own code
#define ecLow 288
#define ecHigh 2042
float ecValue;

// Declarations for Temp Sensor:
const int oneWireBus = 11;     
float temperature;

// Declarations for Water Pressure Sensor:
#define PRESSURE_PIN 21
#define pressure_offset 0.5
#define container_area 0.158 //m^2
float waterAmt;

// Declarations for Water Switch:
#define WATER_SWITCH_PIN 8

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x96, 0xF2, 0x14};  // ! REPLACE WITH YOUR RECEIVER MAC Address

int MAC ;
float pHVal = 0;
float ECVal = 0;
float temp = 0;
float atmtemp = 0;
float hum = 0;
float CO2 = 0;
float Oxy = 0;

typedef struct struct_sensor_reading {
  int MAC ;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
  float atmtemp = 0;
  float hum = 0;
  float CO2 = 0;
  float Oxy = 0;
} struct_sensor_reading;



struct_sensor_reading myData;

esp_now_peer_info_t peerInfo;

// Insert your SSID
constexpr char WIFI_SSID[] = "localize_project";

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
      for (uint8_t i=0; i<n; i++) {
          if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
              return WiFi.channel(i);
          }
      }
  }
  return 0;
}

// put function definitions here:
void phRead(){
  pHValue = analogRead(PH_PIN) * 3.3 / 4096.0;  
  pHValue = scaleOff_ph * pHValue + flatOff_ph;             // Transform avgRead_ph to get pHValue
  Serial.print(pHValue);
  Serial.println(" pH");
}

void ecRead(){
  int j = 0; 
  int V; 
  float grad;
  
  for(int i = 0; i<10; i++){
    j += analogRead(EC_PIN)*3.3/4096;
  }
  V = j/10;

  grad = 11.467/(ecHigh - ecLow);
  ecValue = 12.88 - (ecHigh - V)*grad;

  Serial.print("EC V = ");
  Serial.println(V);
  Serial.print("EC = ");
  Serial.print(ecValue);
  Serial.println("ms/cm^2");
}

void tempRead(){
  sensors.requestTemperatures(); 
  temperature = sensors.getTempCByIndex(0);
  Serial.print(temperature);
  Serial.println("ºC");
}

void waterlevelRead(){
  //calculates amount of water in m^3
  //calculates pressure in kPa (density accounted for in waterAmt calculation)
  float pressure = (analogRead(PRESSURE_PIN) - pressure_offset) * (3.3 / 4096.0) * (1600 / (4.5 - pressure_offset));
  waterAmt = pressure * container_area / 9.81;
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void watermeterOpen() {
  byte command[] = {0x63, 0x05, 0x00, 0x01, 0x00, 0xFF, 0xD4, 0x08};

  CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x63, baud_rate, protocol);
  CWT_Sensor.request_reading(command, 8); 
}

void watermeterClose() {
  byte command[] = {0x63, 0x05, 0x00, 0x01, 0x00, 0x00, 0x94, 0x48};
  CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x63, baud_rate, protocol);
  CWT_Sensor.request_reading(command, 8); 
}

void fillChamber() {
  //Filling of sampling chamber with clean water
  digitalWrite(PERISTALTIC_PIN_3, HIGH);
  delay(PUMP_DURATION);           
  digitalWrite(PERISTALTIC_PIN_3, LOW);
}

// Sequence of events
void sampling_seq() {
    
  // Initial clearing of water
  digitalWrite(HIGH_PERISTALTIC_PIN_2, HIGH);
  delay(CLEAR_DURATION);           
  digitalWrite(HIGH_PERISTALTIC_PIN_2, LOW);
  
  delay(5000);

  // Drawing of sample from mixing tank
  digitalWrite(HIGH_PERISTALTIC_PIN_1, HIGH);
  delay(HIGH_PUMP_DURATION);           
  digitalWrite(HIGH_PERISTALTIC_PIN_1, LOW); 

  tempRead();
  phRead();
  ecRead();

  myData.temp = temperature;
  myData.ECVal = ecValue;
  myData.pHVal = pHValue;

  // Release sample back to mixing tank
  digitalWrite(HIGH_PERISTALTIC_PIN_2, HIGH);
  delay(CLEAR_DURATION);           
  digitalWrite(HIGH_PERISTALTIC_PIN_2, LOW);


  // // Send data to master
  // esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  // if (result == ESP_OK) {
  //   Serial.println("Sent with success");

  // } else {
  //   Serial.println("Error sending the data");
  // }
  // esp_now_register_send_cb(OnDataSent);


  //Adding clean water to chamber
  // digitalWrite(PERISTALTIC_PIN_3, HIGH);
  // delay(PUMP_DURATION);           
  // digitalWrite(PERISTALTIC_PIN_3, LOW); 
}

void dosing_seq () {
  // Get dosing requirements
  // Dosing actuation

  int doseDuration = 1000;
  bool correct_dose = false;
  while (!correct_dose)
  {
    // get_dosing_req();

    // Adding of nutrient A 
    digitalWrite(PERISTALTIC_PIN_1, HIGH);
    delay(doseDuration);           
    digitalWrite(PERISTALTIC_PIN_1, LOW); 

    // Adding of nutrient B
    digitalWrite(PERISTALTIC_PIN_2, HIGH);
    delay(doseDuration);           
    digitalWrite(PERISTALTIC_PIN_2, LOW); 

    // Begin mixing
    digitalWrite(RECIRCULATING_PUMP, HIGH);
    delay(MIXING_DURATION); // Recirculating time
    digitalWrite(RECIRCULATING_PUMP, LOW);

    // Samples to obtain EC and pH values
    sampling_seq();

    // // Check if ONLY EC threshold is met
    // 
    // if() {
    //   correct_dose = true;
    //   break;
    // }
    // else {
    //   continue;
    // }
  }
  

}

// int get_dosing_req() {
//   // Get difference in EC from master

//   // Calculates dosing requirements
//   if () {
//     // if receive
//     return 0;
//   }
//   else {
//     //if no receive
//     return 0;
//   }
// }

// add timer function to start watering every morning

void watering_seq() {
  // 1. Fill mixing tank to full level
  // 2. Dosing Sequence (interatively checks if dosing requirements met, then doses accordingly)
  // 3. Pumps mixture out according to plant daily requirement

  // Fill water level to full
  while(digitalRead(WATER_SWITCH_PIN) == LOW) {
    digitalWrite(IRRIGATION_PUMP, HIGH);
  }

  dosing_seq();

  // Begin irrigation to plants
  digitalWrite(IRRIGATION_PUMP, HIGH);
  delay(IRRIGATION_DURATION); // Irrigation time
  digitalWrite(IRRIGATION_PUMP, LOW);
}

void fillReservoir(){
  while (digitalRead(WATER_SWITCH_PIN) == LOW) {
    digitalWrite(WATER_VALVE, HIGH);
    delay(1000);
  }
}


void setup() {
  Serial.begin(baud_rate);
  // ec.begin();
  sensors.begin();

  Serial.println("Initializing...");

  // memcpy(myData.MAC,MAC_address,17);

  // myData.MAC = 1;

  // WiFi.enableLongRange(true);
  // WiFi.mode(WIFI_STA);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // int32_t channel = getWiFiChannel(WIFI_SSID);
  // while (channel < 1) {
  //   delay(1000);
  //   Serial.println("WiFi Channel Not Found!");
  //   channel = getWiFiChannel(WIFI_SSID);
  // }

  // WiFi.printDiag(Serial); // Uncomment to verify channel number before
  // esp_wifi_set_promiscuous(true);
  // esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  // esp_wifi_set_promiscuous(false);
  // WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // // Init ESP-NOW
  // if (esp_now_init() != ESP_OK) {
  //   Serial.println("Error initializing ESP-NOW");
  // }

  // // Register peer
  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  // peerInfo.channel = 0;
  // peerInfo.encrypt = false;

  // // Add peer
  // if (esp_now_add_peer(&peerInfo) == ESP_OK) {
  //   Serial.println("Peer Added");
  // }


  // Initialize pins
  pinMode(HIGH_PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_2, OUTPUT);
  pinMode(PERISTALTIC_PIN_3, OUTPUT);
  pinMode(RECIRCULATING_PUMP, OUTPUT);
  pinMode(IRRIGATION_PUMP, OUTPUT);
  pinMode(WATER_VALVE, OUTPUT);
  pinMode(HIGH_PERISTALTIC_PIN_2, OUTPUT);

  pinMode(WATER_SWITCH_PIN, INPUT);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(PH_PIN, INPUT);
  pinMode(EC_PIN, INPUT);
  pinMode(oneWireBus, INPUT);

  analogReadResolution(12);


  // sampling_seq();

}

void loop() {

  Serial.println(analogRead(oneWireBus));
  tempRead();

  Serial.println(analogRead(PH_PIN));
  phRead();

  Serial.println(analogRead(EC_PIN));
  ecRead();

  // waterlevelRead();
  delay(10000);
  // digitalWrite(RECIRCULATING_PUMP, HIGH);
  // delay(MIXING_DURATION); // Recirculating time
  // digitalWrite(RECIRCULATING_PUMP, LOW);

  // sampling_seq();
  
  // int32_t channel = getWiFiChannel(WIFI_SSID);
  // channel = getWiFiChannel(WIFI_SSID);
  // while (channel < 1) {
  //   delay(1000);
  //   Serial.println("WiFi Channel Not Found!");
  //   channel = getWiFiChannel(WIFI_SSID);
  // }

  // sampling_seq();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet;
  // esp_now_register_send_cb(OnDataSent);


  // delay(2400000);
  
}



