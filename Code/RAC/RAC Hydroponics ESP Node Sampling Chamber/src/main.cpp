#include <stdint.h>
#include <Arduino.h>

#include <EEPROM.h>
#include <DFRobot_EC.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include "driver/gpio.h"

#include "OneWire.h"
#include "DallasTemperature.h"

// Declarations for Actuation (pumps + valve)
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

// #define PUMP_DURATION 5000
// #define VALVE_DURATION 5000

#define PUMP_DURATION 120000
#define VALVE_DURATION 10000

// #define PUMP_DURATION 480000
// #define VALVE_DURATION 10000

// Change here too!!!
int pumplist[] = 
{
  9,
  10,
  11,
  12,
  13,
  14
};

// Declarations for pH Sensor:
#define PH_PIN 5              // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 15.78      // Flat deviation compensate
#define scaleOff_ph -6        // Scale deviation compensate
float avgRead_ph;             // Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value


// Declarations for EC Sensor:
#define EC_PIN 6

// // Using DFRobot library
// #define flatOff_ec 0.41                     // Flat deviation compensate
// #define scaleOff_ec 1.07                    // Scale deviation compensate
// float voltageRead,ecValue,temperature = 22;
// DFRobot_EC ec;

  // Own code
#define ecLow 215 //1413 microSiemens/cm
#define ecHigh 1922 //12.88 milliSiemens/cm
float ecValue;

// Declarations for Temp Sensor:
const int oneWireBus = 7;     
float temperature;
// char MAC_address[17] = {1,2,3,4,5,6,7,8,9,0,1,2,3,4,5,6,7};

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


uint8_t broadcastAddress[] = {0xEC, 0xDA, 0x3B, 0x96, 0xF2, 0x14};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  int MAC;
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
  // float pHBuffer[10];                                 // Buffer for pH readings
  
  // for(int i = 0; i < 10; i++){                        // Read 10 values for pH
  //   pHBuffer[i] = analogRead(PH_PIN) * 5 / 1024.0;
  // }

  // for(int i = 0; i < 9; i++){                         // Sort pH readings in pHBuffer
  //   for(int j = i + 1; i < 10; j++){
  //     if(pHBuffer[i]>pHBuffer[j]){
  //       float pHBufVal = pHBuffer[i];
  //       pHBuffer[i] = pHBuffer[j];
  //       pHBuffer[j] = pHBufVal;
  //     }
  //   }
  // }

  // for(int i = 2; i < 8; i++){                         // Sum centre six values
  //   avgRead_ph += pHBuffer[i];
  // }

  // avgRead_ph = avgRead_ph/6;                                // Obtain average reading for pH


  pHValue = analogRead(PH_PIN) * 3.3 / 4096.0;  
  pHValue = scaleOff_ph * pHValue + flatOff_ph;             // Transform avgRead_ph to get pHValue
  Serial.print(pHValue);
  Serial.println(" pH");
}

void ecRead(){
    //Using DFRobot Library
  // voltageRead = (analogRead(EC_PIN)*3300)/4096.0;       // Read voltage for EC  
  // ecValue = ec.readEC(voltageRead,temperature);       // Convert voltage to EC Value

  // ecValue = scaleOff_ec * ecValue + flatOff_ec;
  // Serial.print(ecValue);
  // Serial.println(" ms/cm");

    //Using Own code
  int j = 0; 
  int V; 
  float grad;
  
  for(int i = 0; i<10; i++){
    j += analogRead(EC_PIN)*3300/4096;
  }
  V = j/10;
  // Serial.print("Voltage = ");
  // Serial.print(V);
  // Serial.print("V      ");

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

  //float temperatureF = sensors.getTempFByIndex(0);

  Serial.print(temperature);
  Serial.println("ºC");
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Sequence of events
void sampling_seq() {
    
  digitalWrite(VALVE_PIN, HIGH);
  delay(VALVE_DURATION);           
  digitalWrite(VALVE_PIN, LOW);
  
  // for (int i = 0; i < (sizeof(pumplist)) / sizeof(pumplist[0]); i++) {
  for (int i = 1; i < 2; i++) {

    digitalWrite(pumplist[i], HIGH);
    delay(PUMP_DURATION);           
    digitalWrite(pumplist[i], LOW); 

    tempRead();
    phRead();
    ecRead();

    myData.temp = temperature;
    myData.ECVal = ecValue;
    myData.pHVal = pHValue;

    digitalWrite(VALVE_PIN, HIGH);
    delay(VALVE_DURATION);           
    digitalWrite(VALVE_PIN, LOW);

    // digitalWrite(int(PUMP_PIN_6), HIGH);
    // delay(PUMP_DURATION);           
    // digitalWrite(int(PUMP_PIN_6), LOW); 


    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sent with success Pump #" + String(pumplist[i]));

    } else {
      Serial.println("Error sending the data");
    }
    esp_now_register_send_cb(OnDataSent);
  }

  digitalWrite(int(PUMP_PIN_6), HIGH);
  delay(PUMP_DURATION);           
  digitalWrite(int(PUMP_PIN_6), LOW); 

}

void setup() {
  Serial.begin(115200);
  // ec.begin();
  sensors.begin();
  // memcpy(myData.MAC,MAC_address,17);

  myData.MAC = 1;

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  int32_t channel = getWiFiChannel(WIFI_SSID);
  while (channel < 1) {
    delay(1000);
    Serial.println("WiFi Channel Not Found!");
    channel = getWiFiChannel(WIFI_SSID);
  }

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer Added");
  }


  // Initialize pump pins
  pinMode(PUMP_PIN_1, OUTPUT);
  pinMode(PUMP_PIN_2, OUTPUT);
  pinMode(PUMP_PIN_3, OUTPUT);
  pinMode(PUMP_PIN_4, OUTPUT);
  pinMode(PUMP_PIN_5, OUTPUT);
  pinMode(PUMP_PIN_6, OUTPUT);
  pinMode(PUMP_PIN_7, OUTPUT);
  pinMode(VALVE_PIN, OUTPUT);

}

void loop() {
  int32_t channel = getWiFiChannel(WIFI_SSID);
  channel = getWiFiChannel(WIFI_SSID);
  while (channel < 1) {
    delay(1000);
    Serial.println("WiFi Channel Not Found!");
    channel = getWiFiChannel(WIFI_SSID);
  }

  sampling_seq();

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet;
  // esp_now_register_send_cb(OnDataSent);


  // delay(2400000);

  // delay for 15 minutes
  delay(900000);
  
}



