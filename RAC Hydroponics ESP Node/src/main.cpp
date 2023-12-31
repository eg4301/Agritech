#include <stdint.h>
#include <Arduino.h>

#include <EEPROM.h>
#include "DFRobot_EC.h"

#include <Wire.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>



// Declarations for pH Sensor:
<<<<<<< Updated upstream
#define PH_PIN 35          // pH meter Analog output to Arduino Analog Input 0
#define flatOff 0.00       // Flat deviation compensate
#define scaleOff 3.5       // Scale deviation compensate
float avgRead;             //Store the average value of the sensor feedback
float pHValue = 0;         // Final pH Value

#define EC_PIN A1
=======
#define PH_PIN 35             // pH meter Analog output to Arduino Analog Input 0
#define flatOff_ph 0.00       // Flat deviation compensate
#define scaleOff_ph 3.5       // Scale deviation compensate
float avgRead_ph;             //Store the average value of the sensor feedback
float pHValue = 0;            // Final pH Value

// Declarations for EC Sensor:
#define EC_PIN 34
#define flatOff_ec 0.41                     // Flat deviation compensate
#define scaleOff_ec 1.07                    // Scale deviation compensate
>>>>>>> Stashed changes
float voltageRead,ecValue,temperature = 25;
DFRobot_EC ec;

uint8_t broadcastAddress[] = {0xA0, 0xB7, 0x65, 0xFE, 0x85, 0xBC};  // ! REPLACE WITH YOUR RECEIVER MAC Address

typedef struct struct_sensor_reading {
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
} struct_sensor_reading;

struct_sensor_reading myData;

esp_now_peer_info_t peerInfo;

// Insert your SSID
constexpr char WIFI_SSID[] = "Mojave10";

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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ec.begin();

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



// put function definitions here:
void phRead(){
  float pHBuffer[10];                                 // Buffer for pH readings
  
  for(int i = 0; i < 10; i++){                        // Read 10 values for pH
    pHBuffer[i] = analogRead(PH_PIN) * 5 / 1024.0;
  }

  for(int i = 0; i < 9; i++){                         // Sort pH readings in pHBuffer
    for(int j = i + 1; i < 10; j++){
      if(pHBuffer[i]>pHBuffer[j]){
        float pHBufVal = pHBuffer[i];
        pHBuffer[i] = pHBuffer[j];
        pHBuffer[j] = pHBufVal;
      }
    }
  }

  for(int i = 2; i < 8; i++){                         // Sum centre six values
<<<<<<< Updated upstream
    avgRead += pHBuffer[i];
  }

  avgRead = avgRead/6;                                // Obtain average reading for pH

  pHValue = scaleOff * avgRead + flatOff;             // Transform avgRead to get pHValue
=======
    avgRead_ph += pHBuffer[i];
  }

  avgRead_ph = avgRead_ph/6;                                // Obtain average reading for pH

  pHValue = scaleOff_ph * avgRead_ph + flatOff_ph;             // Transform avgRead_ph to get pHValue
>>>>>>> Stashed changes
}

void ecRead(){
  voltageRead = analogRead(EC_PIN)/1024.0*5000;       // Read voltage for EC
  ecValue = ec.readEC(voltageRead,temperature);       // Convert voltage to EC Value

<<<<<<< Updated upstream
=======
  ecValue = scaleOff_ec * ecValue + flatoff_ec;

>>>>>>> Stashed changes
  ec.calibration(voltageRead,temperature);            // Calibrate EC Sensor
}

void tempRead(){
  temperature = 25;                                   // To update when temperature sensor is obtained
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}