// Lilygo ESPNOW receiver

#include <numeric>
#include <vector>
#include <queue>
#include <math.h>
#include <iomanip>
#include <sstream>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ESP32Time.h>
#include <Wire.h>
#include <WiFi.h>
#include "BlynkSimpleTinyGSM.h"
#include "Blynkvpin.h"

// Set your new MAC Address
uint8_t newMACAddress[] = {0x16, 0x16, 0x16, 0x16, 0x16, 0x05};

Sol16TinyGsmSim7600 modem(SerialAT);
#define INF 1e9

// GPRS details
const char apn[] = "";
const char gprs_user[] = "";
const char gprs_pass[] = "";

uint64_t lastreceived;

// Timer
typedef struct struct_sensor_reading {
  byte reading[19];
} struct_sensor_reading;

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

#include <ctime>

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

// MQTT details
const char *broker = "139.59.239.77";
const char *pubTopic = "gateway_to_server";
const char *subTopic = "server_to_gateway";

#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
TinyGsmClient client(modem);
PubSubClient mqtt(client);
uint32_t lastReconnectAttempt = 0;

// database level
String company = "company1";
// blynk auth token (collections level)
String authToken = "HU3aoZ5VcSM9AVOCb9HmmXiWUSiuCxWJ";
String sendVpin = "V0";
double sendData = 0.0;
bool is_send_data = false;

boolean mqttConnect()
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect(broker);

  // Or, if you want to authenticate MQTT:

  if (status == false)
  {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" success");

  mqtt.subscribe(subTopic);
  return mqtt.connected();
}

void mqttCallback(char *topic, byte *payload, unsigned int len)
{
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  // if (String(topic) == topicLed)
  // {
  // ledStatus = !ledStatus;
  // digitalWrite(LED_PIN, ledStatus);
  // SerialMon.print("ledStatus:");
  // SerialMon.println(ledStatus);
  // mqtt.publish(topicLedStatus, "1");
  // }
}

void mqttPublish(String company, String authToken, int timestamp, String vpin, double value)
{
  StaticJsonDocument<200> doc;
  doc["company"] = company;
  doc["authToken"] = authToken;
  doc["timestamp"] = timestamp;
  doc["vpin"] = vpin;
  doc["value"] = value;
  char jsonBuffer[512];
  serializeJson(doc, jsonBuffer); // print to client

  mqtt.publish(pubTopic, jsonBuffer);
}

String random_string(int len)
{
  static const char alphanum[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";

  String out = "";

  for (int i = 0; i < len; i += 1)
  {
    out += alphanum[rand() % (sizeof(alphanum) - 1)];
  }

  return out;
}

void updateTime()
{
  // cli();
  timestamp = modem.get_str_datetime();
  // sei();;
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  // cli();
  memcpy(&incoming_data, incomingData, sizeof(incoming_data));
  int vpin = incoming_data.vpin * 16;
  double battvoltage = incoming_data.battvoltage;

  timestamp = get_formatted_time();
  timestamp_datas.push({vpin + 0, timestamp});
  counter_datas.push({vpin + 1, 0}); // 0 used as placeholder, will sync and update before sending to Blynk
  battvoltage_datas.push({vpin + 2, battvoltage});

  lastreceived = millis();
  for (int i = 0; i < 13; i++)
  {
    if (incoming_data.data[i] != INF)
    {
      new_datas.push({vpin + i + 3, incoming_data.data[i]});
      Serial.println("New data received: " + String(vpin + i + 3) + " " + String(incoming_data.data[i]));
      // Blynk.virtualWrite(vpin + i + 3, (double)incoming_data.data[i]);

      sendVpin = "V" + (String)(vpin + i + 3);
      sendData = (double)incoming_data.data[i];

      is_send_data = true;
      delay(500);
    }
  }
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
  lastreceived = millis();
  Serial.begin(115200, SERIAL_8N1); // Start the built-in serial port
  modem.setup();
  DBG("Completed modem setup");
  delay(1000);

  setupRTC();

  WiFi.enableLongRange(true);
  WiFi.mode(WIFI_STA);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  
  Serial.print("[OLD] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  Serial.print("[NEW] ESP32 Board MAC Address:  ");
  Serial.println(WiFi.macAddress());
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  DBG("Lilygo MAC address:", WiFi.macAddress());

  // Blynk.begin(auth, modem, apn, gprs_user, gprs_pass);
  delay(100);
  // timer.setInterval(1500L, sendBlynk);
  // timer.setInterval(5000L, maintainConnection);
  // timer.setInterval(1000L, updateTime);

  esp_now_register_recv_cb(OnDataRecv);

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  // mqtt.setCallback(mqttCallback);

  while (!mqtt.connected())
  {
    SerialMon.println("=== MQTT NOT CONNECTED ===");

    mqttConnect();
  }
}

void loop()
{
  // Blynk.run();
  // timer.run();

  if (!mqtt.connected())
  {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    uint32_t t = millis();
    if (t - lastReconnectAttempt > 5000L)
    {
      lastReconnectAttempt = t;
      if (mqttConnect())
      {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
  else if (is_send_data)
  {
    mqttPublish(getUnixTime(timestamp), sendVpin, sendData);
    is_send_data = false;
  }

  mqtt.loop();
}