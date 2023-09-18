/**************************************************************
 *
 * For this example, you need to install PubSubClient library:
 *   https://github.com/knolleary/pubsubclient
 *   or from http://librarymanager/all#PubSubClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more MQTT examples, see PubSubClient library
 *
 **************************************************************
 * This example connects to HiveMQ's showcase broker.
 *
 * You can quickly test sending and receiving messages from the HiveMQ webclient
 * available at http://www.hivemq.com/demos/websocket-client/.
 *
 * Subscribe to the topic GsmClientTest/ledStatus
 * Publish "toggle" to the topic GsmClientTest/led and the LED on your board
 * should toggle and you should see a new message published to
 * GsmClientTest/ledStatus with the newest LED status.
 *
 **************************************************************/

#define TINY_GSM_MODEM_SIM7600  //A7670's AT instruction is compatible with SIM7600

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#define SerialAT Serial1

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
#define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false


// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
// const char apn[]      = "";
// const char gprsUser[] = "";
// const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
// const char wifiSSID[] = "YourSSID";
// const char wifiPass[] = "YourWiFiPass";

// MQTT details
#define AWS_IOT_PUBLISH_TOPIC   "esp32/pub"
#define AWS_IOT_SUBSCRIBE_TOPIC "esp32/sub"

// MQTT topics
const char *topicLed       = "GsmClientTest/led";
const char *topicInit      = "GsmClientTest/init";
const char *topicLedStatus = "GsmClientTest/ledStatus";

#include <numeric>
#include <vector>
#include <queue>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <TinyGsmClient.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <WiFi.h>
#include <ctime>
#include <queue>
#include "AWS_Certs.h"



// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif
TinyGsmClient client(modem);
PubSubClient  mqtt(client);



Ticker tick;


#define uS_TO_S_FACTOR          1000000ULL  /*  Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP           60          /*Time ESP32 will go to sleep (in seconds) */

#define UART_BAUD    115200
#define PIN_DTR      25
#define PIN_TX       26
#define PIN_RX       27
#define PWR_PIN      4
#define BAT_ADC      35
#define BAT_EN       12
#define PIN_RI       33
#define RESET        5

#define SD_MISO     2
#define SD_MOSI     15
#define SD_SCLK     14
#define SD_CS       13

int ledStatus = LOW;
// Set your new MAC Address
uint8_t newMACAddress[] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x05};

uint32_t lastReconnectAttempt = 0;
uint16_t combinedhex = 0;
uint16_t lastreceived;
std::queue<uint16_t> humidity = {}; // queue for humidity values
std::queue<uint16_t> temp = {};     // queue for temperature values 
std::queue<uint16_t> conduct = {};  // queue for conductivity values
std::queue<uint16_t> pH = {};       // queue for pH values
std::queue<uint16_t> N = {};        // queue for nitrogen values
std::queue<uint16_t> P = {};        // queue for phosphorous values
std::queue<uint16_t> K = {};        // queue for potassium values
uint16_t hum_now = 0;
uint16_t temp_now = 0;
uint16_t con_now = 0;
uint16_t pH_now = 0;
uint16_t N_now = 0;
uint16_t P_now = 0;
uint16_t K_now = 0;
bool is_send_data = false;
String timestamp;
WiFiClientSecure net = WiFiClientSecure();

void connectAWS(){
  // Configure WiFiClientSecure to use AWS IoT device credentials.
  net.setCACert(AWS_CERT_CA);
  net.setCertificate(AWS_CERT_CRT);
  net.setPrivateKey(AWS_CERT_PRIVATE);

  // Connect to the MQTT Broker on the AWS endpoint we defined earlier.
  mqtt.setServer(AWS_IOT_ENDPOINT, 8883);

  //Create a message handler
  mqtt.setCallback(mqttCallback);

  Serial.println("Connecting to AWS IOT");
 
  while (!mqtt.connect(THINGNAME))
  {
    Serial.print(".");
    delay(100);
  }
 
  if (!mqtt.connected())
  {
    Serial.println("AWS IoT Timeout!");
    return;
  }
 
  // Subscribe to a topic
  mqtt.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
 
  Serial.println("AWS IoT Connected!");
}

// RTC
struct tm timeinfo;
float timezone;
ESP32Time rtc;
uint64_t seconds, minutes, hours, days, months, year;


/*
 * Time Functions
 */

/**
 * @brief Setup the RTC using datetime received from modem AT commands. modem needs to be done setting up and initializing before this is called.
 *
 */
void setupRTC()
{
  // Get the current date and time from the network
  modem.getNetworkTime(&timeinfo.tm_year, &timeinfo.tm_mon, &timeinfo.tm_mday,
                       &timeinfo.tm_hour, &timeinfo.tm_min, &timeinfo.tm_sec, &timezone);

  rtc.setTime(timeinfo.tm_sec, timeinfo.tm_min, timeinfo.tm_hour, timeinfo.tm_mday, timeinfo.tm_mon, timeinfo.tm_year);
}

/**
 * @brief Returns the current datetime in a formatted string.
 */
String get_formatted_time()
{
  return rtc.getTime("%d %B %Y %I:%M:%S %p");
}



int getUnixTime(String time_string)
{
  // Set up a tm structure to hold the time
  tm time_tm;

  // Use a stringstream to parse the time string
  std::istringstream time_stream(time_string.c_str());

  // Use the stream to parse the time into the tm structure
  time_stream >> std::get_time(&time_tm, "%d %B %Y %I:%M:%S %p");

  // Convert the tm structure to a time_t
  time_t time_t = std::mktime(&time_tm);

  // Convert the time_t to an int and return it
  return static_cast<int>(time_t);
}

void hexconcat(uint16_t HEX_A, uint16_t HEX_B){
  combinedhex = HEX_A;
  combinedhex <<= 8;
  combinedhex = combinedhex | HEX_B;
  
}

void mqttPublish(int timestamp, uint16_t hum_now, uint16_t temp_now, uint16_t con_now, uint16_t pH_now, uint16_t N_now, uint16_t P_now, uint16_t K_now)
{
  StaticJsonDocument<200> doc;
  doc["timestamp"] = timestamp;
  doc["humidity"] = hum_now;
  doc["temperature"] = temp_now;
  doc["conductivity"] = con_now;
  doc["pH"] = pH_now;
  doc["Nitrogen"] = N_now;
  doc["Phosphorous"] = P_now;
  doc["Potassium"] = K_now;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  mqtt.publish(AWS_IOT_PUBLISH_TOPIC, jsonBuffer);
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{

    SerialMon.print("Message arrived [");
    SerialMon.print(topic);
    SerialMon.print("]: ");
    SerialMon.write(payload, len);
    SerialMon.println();

    // Only proceed if incoming message's topic matches
    if (String(topic) == topicLed) {
        ledStatus = !ledStatus;
        // digitalWrite(LED_PIN, ledStatus);
        SerialMon.print("ledStatus:");
        SerialMon.println(ledStatus);
        mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");

    }
}

boolean mqttConnect()
{
    SerialMon.print("Connecting to AWS");

    // Connect to MQTT Broker
    boolean status = mqtt.connect(THINGNAME);

    // Or, if you want to authenticate MQTT:
    // boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

    if (status == false) {
        SerialMon.println(" fail");
        return false;
    }
    SerialMon.println(" success");
    mqtt.publish(AWS_IOT_PUBLISH_TOPIC, "Master Hub Pub");
    mqtt.subscribe(AWS_IOT_SUBSCRIBE_TOPIC);
    return mqtt.connected();
}

typedef struct struct_sensor_reading {
  byte reading[19];
} struct_sensor_reading;

struct_sensor_reading incoming_data;


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // cli();
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  timestamp = get_formatted_time();
  lastreceived = millis();

  if ((incoming_data.reading[0] == 1) && (incoming_data.reading[1] == 3)){
    
    hexconcat(incoming_data.reading[3],incoming_data.reading[4]);
    humidity.push(combinedhex);
    Serial.println(combinedhex);
    
    hexconcat(incoming_data.reading[5],incoming_data.reading[6]);
    temp.push(combinedhex);
    Serial.println(combinedhex);
    
    hexconcat(incoming_data.reading[7],incoming_data.reading[8]);
    conduct.push(combinedhex);
    Serial.println(combinedhex);
    
    hexconcat(incoming_data.reading[9],incoming_data.reading[10]);
    pH.push(combinedhex);
    Serial.println(combinedhex);
    
    hexconcat(incoming_data.reading[11],incoming_data.reading[12]);
    N.push(combinedhex);
    Serial.println(combinedhex);

    hexconcat(incoming_data.reading[13],incoming_data.reading[14]);
    P.push(combinedhex);
    Serial.println(combinedhex);

    hexconcat(incoming_data.reading[15],incoming_data.reading[16]);
    K.push(combinedhex);
    Serial.println(combinedhex);

    combinedhex = 0; // reset combinedhex value
  }
  
  // for (int i = 0; i < 13; i++)
  // {
  //   if (incoming_data.data[i] != INF)
  //   {
  //     new_datas.push({vpin + i + 3, incoming_data.data[i]});
  //     Serial.println("New data received: " + String(vpin + i + 3) + " " + String(incoming_data.data[i]));
  //     // Blynk.virtualWrite(vpin + i + 3, (double)incoming_data.data[i]);

  //     sendVpin = "V" + (String)(vpin + i + 3);
  //     sendData = (double)incoming_data.data[i];

  //     is_send_data = true;
  //     delay(500);
  //   }
  // }
  // int data = incoming_data.data;
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("VPIN: ");
  // Serial.println(vpin);
  // Serial.print("Data: ");
  // Serial.println(data);
  // Serial.println();
  // Blynk.virtualWrite(vpin, data);
  // new_datas.push_back({vpin, data});
  // sei();;
}



void setup()
{
    // Set console baud rate
    Serial.begin(115200);
    delay(10);

    pinMode(BAT_EN, OUTPUT);
    digitalWrite(BAT_EN, HIGH);

    //A7670 Reset
    pinMode(RESET, OUTPUT);
    digitalWrite(RESET, LOW);
    delay(100);
    digitalWrite(RESET, HIGH);
    delay(3000);
    digitalWrite(RESET, LOW);

    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, LOW);
    delay(100);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_PIN, LOW);




    Serial.println("\nWait...");

    delay(10000);

    SerialAT.begin(UART_BAUD, SERIAL_8N1, PIN_RX, PIN_TX);


    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    DBG("Initializing modem...");
    if (!modem.init()) {
        DBG("Failed to restart modem, delaying 10s and retrying");
        return;
    }
    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    DBG("Initializing modem...");
    if (!modem.restart()) {
        DBG("Failed to restart modem, delaying 10s and retrying");
        return;
    }


    // String name = modem.getModemName();
    // DBG("Modem Name:", name);

    // String modemInfo = modem.getModemInfo();
    // DBG("Modem Info:", modemInfo);

#if TINY_GSM_USE_GPRS
    // Unlock your SIM card with a PIN if needed
    if (GSM_PIN && modem.getSimStatus() != 3) {
        modem.simUnlock(GSM_PIN);
    }
#endif

// #if TINY_GSM_USE_WIFI
//     // Wifi connection parameters must be set before waiting for the network
//     SerialMon.print(F("Setting SSID/password..."));
//     if (!modem.networkConnect(wifiSSID, wifiPass)) {
//         SerialMon.println(" fail");
//         delay(10000);
//         return;
//     }
//     SerialMon.println(" success");
// #endif

// #if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
//     // The XBee must run the gprsConnect function BEFORE waiting for network!
//     modem.gprsConnect(apn, gprsUser, gprsPass);
// #endif

//     SerialMon.print("Waiting for network...");
//     if (!modem.waitForNetwork()) {
//         SerialMon.println(" fail");
//         delay(10000);
//         return;
//     }
//     SerialMon.println(" success");

//     if (modem.isNetworkConnected()) {
//         SerialMon.println("Network connected");
//     }

// #if TINY_GSM_USE_GPRS
//     // GPRS connection parameters are usually set after network registration
//     SerialMon.print(F("Connecting to "));
//     SerialMon.print(apn);
//     if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
//         SerialMon.println(" fail");
//         delay(10000);
//         return;
//     }
//     SerialMon.println(" success");

//     if (modem.isGprsConnected()) {
//         SerialMon.println("GPRS connected");
//     }
// #endif
  delay(1000);

  setupRTC();

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  // Serial.print("[OLD] ESP32 Board MAC Address:  ");
  // Serial.println(WiFi.macAddress());
  // esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // Serial.print("[NEW] ESP32 Board MAC Address:  ");
  // Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  connectAWS();

}

void loop()
{


    // Make sure we're still registered on the network
    if (!modem.isNetworkConnected()) {
        SerialMon.println("Network disconnected");
        if (!modem.waitForNetwork(180000L, true)) {
            SerialMon.println(" fail");
            delay(10000);
            return;
        }
        if (modem.isNetworkConnected()) {
            SerialMon.println("Network re-connected");
        }
      }
    
    if (!mqtt.connected()) {
        SerialMon.println("=== MQTT NOT CONNECTED ===");
        // Reconnect every 10 seconds
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 10000L) {
            lastReconnectAttempt = t;
            if (mqttConnect()) {
                lastReconnectAttempt = 0;
            }
        }
        delay(100);
        return;
    }
    else if (is_send_data)
    {
    mqttPublish(getUnixTime(timestamp), hum_now, temp_now, con_now, pH_now, N_now, P_now, K_now);
    is_send_data = false;
    }

    mqtt.loop();

}


