#include <stdint.h>
#include <Arduino.h>

#include <Wire.h>
#include "esp_wifi.h"
#include "esp_adc_cal.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Preferences.h>

#include "Sol16_RS485.h"
#include "SoftwareSerial.h"

/* Public defines ----------------------------------------------------------- */
#define DEEPSLEEPDURATION (24 * 60 * 60)  // Time interval between readings, in seconds (default 24 hours)
#define ADDRESS (0x01)                    // ! NEED TO CHANGE FOR EACH WATER METER

#define EN_1 12

// RS485 pins in use
#define RX_PIN 16    // Soft Serial Receive pin, connected to RO // ! PINOUT TBC ONCE PCB ARRIVES
#define TX_PIN 17    // Soft Serial Transmit pin, connected to DI // ! PINOUT TBC ONCE PCB ARRIVES
#define CTRL_PIN 26  // RS485 Direction control, connected to RE and DE // ! PINOUT TBC ONCE PCB ARRIVES

#define baud_rate 9600

// RS485 Constants
#define RS485_TRANSMIT HIGH
#define RS485_RECEIVE LOW

// Soil sensor constants
#define totSensors 6  // Total number of sensors
#define numReadingTypes 5 // Number of readings types to be taken
#define numReadings 5 // Number of readings to be taken

Config protocol = SWSERIAL_8N1;

Preferences preferences;

Sol16_RS485Sensor CWT_Sensor(RX_PIN, TX_PIN);

// CWT sensor addresses
byte hexI[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08, 0x09, 0x10};
byte reading[19];

byte sensorTransform[totSensors][numReadingTypes][3] {};
// sensorTrends[i][j][0] = c1;
// sensorTrends[i][j][1] = m2/m1;
// sensorTrends[i][j][2] = c2;
// readings[i][0] = address;
// readings[i][1] = humidity(%);
// readings[i][2] = temperature(C);
// readings[i][3] = conductivity(us/cm);
// readings[i][4] = PH;

/* Private Constants -------------------------------------------------------- */
uint8_t broadcastAddress[] = {0x48, 0x27, 0xE2, 0x61, 0x8F, 0x58};  // ! REPLACE WITH YOUR RECEIVER MAC Address

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

float temp_now = 0;
float con_now = 0;
float pH_now = 0;
float atmtemp_now = 0;
float hum_now = 0;
float CO2_now = 0;
float oxy_now = 0;
float N_now = 0;
float P_now = 0;
float K_now = 0;
int MAC_now= 0;

typedef struct struct_sensor_reading {
  int MAC = 0;
  float pHVal = 0;
  float ECVal = 0;
  float temp = 0;
  float atmtemp = 0;
  float hum = 0;
  float CO2 = 0;
  float Oxy = 0;
  float N = 0;
  float P = 0;
  float K = 0;
} struct_sensor_reading;

struct_sensor_reading myData;

esp_now_peer_info_t peerInfo;



void request_reading_CWT(byte address);
void request_reading_rika();
void receive_reading(byte reading[], int num_bytes);
void packDataCWT(byte reading[]);
void packDataRika(byte reading[]);

void retrieveTransform(byte sensorTransform[totSensors][numReadingTypes][3]);
int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val);


void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {

  Serial.begin(9600);



  pinMode(EN_1, OUTPUT);
  digitalWrite(EN_1, HIGH);
  delay(100);

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
  gpio_hold_dis(GPIO_NUM_2); 
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  int32_t channel = getWiFiChannel(WIFI_SSID);
  channel = getWiFiChannel(WIFI_SSID);
  while (channel < 1) {
    delay(1000);
    Serial.println("WiFi Channel Not Found!");
    channel = getWiFiChannel(WIFI_SSID);
  }

  for (int i=1; i<6; i++){

    if (i<5){
    // Read and send CWT data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, hexI[i], baud_rate, protocol);  // Serial connection setup
    request_reading_CWT(hexI[i]);
    receive_reading(reading, 19);
    packDataCWT(reading);
    }    

    else {
    // Read and send Rika data
    CWT_Sensor.setup(CTRL_PIN, RX_PIN, TX_PIN, 0x06, baud_rate, protocol);  // Serial connection setup
    request_reading_rika();
    receive_reading(reading, 13);
    packDataRika(reading);
    }     

    myData.MAC = MAC_now;
    myData.temp = temp_now;
    myData.ECVal = con_now;
    myData.pHVal = pH_now;
    myData.hum = hum_now;
    myData.N = N_now;
    myData.P = P_now;
    myData.K = K_now;
    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    } else {
      Serial.println("Error sending the data");
    }
    // Once ESPNow is successfully Init, we will register for Send CB to
    // get the status of Trasnmitted packet
    esp_now_register_send_cb(OnDataSent);

    delay(5000);
  }
  
  // 10min delay between readings

  Serial.println("Go to deep sleep");
  // Turn off and keep off the built-in led during deep sleep
  digitalWrite(LED_BUILTIN, LOW);   
  gpio_hold_en(GPIO_NUM_2); 

  // Set deep sleep duration (3600s = 1hr)
  esp_sleep_enable_timer_wakeup(300 * 1000000ULL); 

  // Switch to deep sleep mode
  esp_deep_sleep_start(); 

}

void request_reading_CWT(byte address) {
  byte command[8] ={address, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}

void request_reading_rika() {
  byte command[8] = {0x06, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09};
  CWT_Sensor.add_crc(command, 0, 6);
  CWT_Sensor.request_reading(command, 8);
}

void receive_reading(byte reading[], int num_bytes) {
  CWT_Sensor.receive_reading(num_bytes, RETURN_ADDRESS_IDX, RETURN_FUNCTIONCODE_IDX, reading);
}

void packDataCWT(byte reading[]) {
  delay(5000);

  MAC_now = (int)(reading[0]);
  // hum_now = transform(sensorTransform, MAC_now, 1, (reading[3] << 8 | reading[4]) / 10);
  // con_now = transform(sensorTransform, MAC_now, 3, (reading[7] << 8 | reading[8]));
  // pH_now = transform(sensorTransform, MAC_now, 4, (reading[9] << 8 | reading[10]) / 10);
  // temp_now = transform(sensorTransform, MAC_now, 2, (reading[5] << 8 | reading[6]) / 10);
  hum_now = (reading[3] << 8 | reading[4]) / 10;
  con_now = (reading[7] << 8 | reading[8]);
  pH_now = (reading[9] << 8 | reading[10]) / 10;
  temp_now = (reading[5] << 8 | reading[6]) / 10;
  N_now = (reading[11] << 8 | reading[12]) / 10;
  P_now = (reading[13] << 8 | reading[14]) / 10;
  K_now = (reading[15] << 8 | reading[16]) / 10;
}

void packDataRika(byte reading[]) {
  delay(5000);

  MAC_now = 6;
  temp_now = (reading[3] << 8 | reading[4]) / 10;
  con_now = (reading[7] << 8 | reading[8]);
  pH_now = (reading[9] << 8 | reading[10]) / 100;
  hum_now = (reading[5] << 8 | reading[6]) / 10;
}

void retrieveTransform(byte sensorTransform[totSensors][numReadingTypes][3]) {
  preferences.begin("Transform", true);

  for (int sensor = 0; sensor < totSensors; sensor++) {
      for (int readingType = 1; readingType < numReadingTypes; readingType++) {
      // Construct the key as "sensori_j_0" and "sensor_i_j_1" for slope and intercept respectively
      String keyC1 = "sensor" + String(sensor) + "_" + String(readingType) + "0";
      String keym2_m1 = "sensor" + String(sensor) + "_" + String(readingType) + "1";
      String keyC2 = "sensor" + String(sensor) + "_" + String(readingType) + "2";

      // Store the slope and intercept values
      sensorTransform[sensor][readingType][0] = preferences.getFloat(keyC1.c_str(), 1);
      sensorTransform[sensor][readingType][1] = preferences.getFloat(keym2_m1.c_str(), 1);
      sensorTransform[sensor][readingType][2] = preferences.getFloat(keyC2.c_str(), 1);
    }
  }

  preferences.end();
}

int transform(byte sensorTransform[totSensors][numReadingTypes][3], int sensNum, int readingType, float val) {
  // y2 = (y1-c1) x (m2/m1) + c2
  return (val - sensorTransform[sensNum][readingType][0]) * sensorTransform[sensNum][readingType][1] + sensorTransform[sensNum][readingType][2];
}