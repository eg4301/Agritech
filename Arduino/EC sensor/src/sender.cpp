/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-esp-now-wi-fi-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Arduino.h>
#include <DFRobot_SHT3x.h>

//DFRobot_SHT3x sht3x(&Wire,/*address=*/0x45,/*RST=*/4);
DFRobot_SHT3x   sht3x;

//sensor calibration
const int AirValue = 3700;   //you need to change this value that you had recorded in the air
const int WaterValue = 500;  //you need to change this value that you had recorded in the water
const int InputPin = 35;

int intervals = (AirValue - WaterValue)/3;
int soilMoistureValue = 0;

// Set your Board ID (ESP32 Sender #1 = BOARD_ID 1, ESP32 Sender #2 = BOARD_ID 2, etc)
#define BOARD_ID 1

//MAC Address of the receiver 
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};



//Structure example to send data
//Must match the receiver structure
typedef struct struct_message {
    int id;
    float temp;
    float hum;
    int readingId;
} struct_message;

//Create a struct_message called myData
struct_message myData;

unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings

unsigned int readingId = 0;

// Insert your SSID
constexpr char WIFI_SSID[] = "REPLACE_WITH_YOUR_SSID";

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

float readTemp() {

  float temp = sht3x.getTemperatureC();
  Serial.println(String(temp) + "C");
  return temp;

}

float readHum() {

  float humidity = sht3x.getHumidityLowSetRH();
  Serial.println(String(humidity) + "%RH");
  return humidity;

}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
 
void setup() {
  //Init Serial Monitor
  Serial.begin(115200);

 
  // Set device as a Wi-Fi Station and set channel
  WiFi.mode(WIFI_STA);

  int32_t channel = getWiFiChannel(WIFI_SSID);

  WiFi.printDiag(Serial); // Uncomment to verify channel number before
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  WiFi.printDiag(Serial); // Uncomment to verify channel change after

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  //Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.encrypt = false;
  
  //Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }


  // sht3x initialize
  while (sht3x.begin() != 0) {
    Serial.println("Failed to Initialize the chip, please confirm the wire connection");
    delay(1000);
  }
  /**
   * readSerialNumber Read the serial number of the chip.
   * @return Return 32-digit serial number.
   */
  Serial.print("Chip serial number");
  Serial.println(sht3x.readSerialNumber());

  /**
   * softReset Send command resets via IIC, enter the chip's default mode single-measure mode, 
   * turn off the heater, and clear the alert of the ALERT pin.
   * @return Read the register status to determine whether the command was executed successfully, 
   * and return true indicates success.
   */
   if(!sht3x.softReset()){
     Serial.println("Failed to Initialize the chip....");
   }

  /**
   * heaterEnable(): Turn on the heater inside the chip to enable the sensor get correct humidity value in wet environments.
   * @return Read the status of the register to determine whether the command was executed successfully,
   * and return true indicates success.
   * @note Heaters should be used in wet environments, and other cases of use will result in incorrect readings
   */

  //if(!sht3x.heaterEnable()){
  // Serial.println("Failed to turn on the heater....");
  //}
  Serial.println("------------------Read data in single measurement mode-----------------------");


}
 
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    // Save the last time a new reading was published
    previousMillis = currentMillis;
    //Set values to send
    myData.id = BOARD_ID;
    myData.temp = readTemp();
    myData.hum = readHum();
    myData.readingId = readingId++;
     
    //Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
  }
}