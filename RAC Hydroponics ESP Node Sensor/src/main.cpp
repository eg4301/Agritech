#include <stdint.h>
#include <Arduino.h>

#include <EEPROM.h>
#include <DFRobot_EC.h>

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>

#include "OneWire.h"
#include "DallasTemperature.h"



// Declarations for pH Sensor:
#define PH_PIN 5             // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 16.06       // Flat deviation compensate
#define scaleOff_ph -6       // Scale deviation compensate
float avgRead_ph;             //Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value

// Declarations for EC Sensor:
#define EC_PIN 6
#define flatOff_ec 0.41                     // Flat deviation compensate
#define scaleOff_ec 1.07                    // Scale deviation compensate
float voltageRead,ecValue,temperature = 22;
DFRobot_EC ec;

// Declarations for Temp Sensor:
const int oneWireBus = 7;     

OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);


uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x61, 0x8F, 0x58};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  int MAC;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
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
  voltageRead = (analogRead(EC_PIN)*3300)/4096.0;       // Read voltage for EC
  ecValue = ec.readEC(voltageRead,temperature);       // Convert voltage to EC Value

  ecValue = scaleOff_ec * ecValue + flatOff_ec;
  Serial.print(ecValue);
  Serial.println(" ms/cm");
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ec.begin();
  sensors.begin();
  myData.MAC = 1;

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer Added");
    return;
  }
}

void loop() {
  tempRead();
  Serial.println(temperature);
  phRead();
  Serial.println(pHValue);
  ecRead();
  Serial.println(ecValue);

  myData.temp = temperature;
  myData.ECVal = ecValue;
  myData.pHVal = pHValue;



  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  delay(30000);
  
}



