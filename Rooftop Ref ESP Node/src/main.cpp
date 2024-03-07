#include <stdint.h>
#include <Arduino.h>

#include <DFRobot_EC.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include "driver/gpio.h"

#include "OneWire.h"
#include "DallasTemperature.h"


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
// When changin pins, please also change the pin numbers manually in pumplist!!!
#define HIGH_PERISTALTIC_PIN_1 4
#define PERISTALTIC_PIN_1 5
#define PERISTALTIC_PIN_2 6
#define PERISTALTIC_PIN_3 7
#define RECIRCULATING_PUMP 15
#define IRRIGATION_PUMP 16
#define WATER_VALVE 17
#define SAMPLING_VALVE 18

// Define length of time pumps and valves are open

// #define PUMP_DURATION 5000
// #define VALVE_DURATION 5000

#define PUMP_DURATION 120000
#define HIGH_PUMP_DURATION 12000
#define VALVE_DURATION 10000


// #define PUMP_DURATION 480000
// #define VALVE_DURATION 10000



// Declarations for pH Sensor:
#define PH_PIN 5             // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 15.78       // Flat deviation compensate
#define scaleOff_ph -6       // Scale deviation compensate
float avgRead_ph;             //Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value


// Declarations for EC Sensor:
#define EC_PIN 6

  // Using DFRobot library
// #define flatOff_ec 0.41                     // Flat deviation compensate
// #define scaleOff_ec 1.07                    // Scale deviation compensate
// float voltageRead,ecValue,temperature = 22;
// DFRobot_EC ec;

  // Own code
#define ecLow 215
#define ecHigh 1922
float ecValue;

// Declarations for Temp Sensor:
const int oneWireBus = 7;     
float temperature;

// Declarations for Water Pressure Sensor:
#define PRESSURE_PIN 8
#define pressure_offset 0.5
#define container_area 1
float waterAmt;

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
  Serial.print(temperature);
  Serial.println("ºC");
}

void waterlevelRead(){
  //calculates pressure in kPa (density accounted for in waterAmt calculation)
  float pressure = (analogRead(PRESSURE_PIN) - pressure_offset) * (3.3 / 4096.0) * (1600 / (4.5 - pressure_offset));
  waterAmt = pressure * container_area;
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
  digitalWrite(SAMPLING_VALVE, HIGH);
  delay(VALVE_DURATION);           
  digitalWrite(SAMPLING_VALVE, LOW);
  
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
  digitalWrite(SAMPLING_VALVE, HIGH);
  delay(VALVE_DURATION);           
  digitalWrite(SAMPLING_VALVE, LOW);


  // Send data to master
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success Pump #" + String(pumplist[i]));

  } else {
    Serial.println("Error sending the data");
  }
  esp_now_register_send_cb(OnDataSent);


  //Clearing of sampling chamber
  digitalWrite(SAMPLING_VALVE, HIGH);
  delay(VALVE_DURATION);           
  digitalWrite(SAMPLING_VALVE, LOW); 
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
  pinMode(HIGH_PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_1, OUTPUT);
  pinMode(PERISTALTIC_PIN_2, OUTPUT);
  pinMode(PERISTALTIC_PIN_3, OUTPUT);
  pinMode(RECIRCULATING_PUMP, OUTPUT);
  pinMode(IRRIGATION_PUMP, OUTPUT);
  pinMode(WATER_VALVE, OUTPUT);
  pinMode(SAMPLING_VALVE, OUTPUT);

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
  delay(900000);
  
}



