/*************************************************************
  Download latest Blynk library here:
    https://github.com/blynkkk/blynk-library/releases/latest
  Blynk is a platform with iOS and Android apps to control
  Arduino, Raspberry Pi and the likes over the Internet.
  You can easily build graphic interfaces for all your
  projects by simply dragging and dropping widgets.
    Downloads, docs, tutorials: http://www.blynk.cc
    Sketch generator:           http://examples.blynk.cc
    Blynk community:            http://community.blynk.cc
    Follow us:                  http://www.fb.com/blynkapp
                                http://twitter.com/blynk_app
  Blynk library is licensed under MIT license
  This example code is in public domain.
 *************************************************************
  This example runs directly on ESP32 chip.
  Note: This requires ESP32 support package:
    https://github.com/espressif/arduino-esp32
  Please be sure to select the right ESP32 module
  in the Tools -> Board menu!
  Change WiFi ssid, pass, and Blynk auth token to run :)
  Feel free to apply it to any other example. It's simple!
 *************************************************************/

/* Comment this out to disable prints and save space */



#define BLYNK_PRINT Serial

#define RELAY_PIN_1 21
#define RELAY_PIN_2 19
#define RELAY_PIN_3 18
#define RELAY_PIN_4 5

#define LED_PIN     25

#define WATER_POWER_PIN  27
#define WATER_SENS_PIN 23

#define WATER_SWITCH_PIN  22

int water_value = 0; // variable to store the water sensor value

int water_level = 0;

int PUMP_DURATION_4;
int PUMP_DURATION_3;

// weather api definition
WiFiClientSecure client;

const char* host = "https://api.data.gov.sg";
const int httpsPort = 443; // for HTTPS requests

char forecast = "NA";


/* Fill-in your Template ID (only if using Blynk.Cloud) */
#define BLYNK_TEMPLATE_ID "TMPL65P0inFvF"
#define BLYNK_TEMPLATE_NAME "T Relay"
#define BLYNK_AUTH_TOKEN "ECciLuWMdGzXaKDVJnmbU-lTbPw0p4-r"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#include <ArduinoJson.h>
#include <SPI.h>

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your WiFi credentials.
// Set password to "" for open networks.
// char ssid[] = "SINGTEL-1F2B";
// char pass[] = "owaibohvae";

char ssid[] = "localize_project";
char pass[] = "localize_project";

// char ssid[] = "wz";
// char pass[] = "Possword1.";

BLYNK_WRITE(V0)
{
  if (param.asInt() == 1) {
    digitalWrite(RELAY_PIN_1, HIGH);
    Blynk.logEvent("relay_state", "RELAY_1 ON");//Sending Events
  } else {
    digitalWrite(RELAY_PIN_1,  LOW);
    Blynk.logEvent("relay_state", "RELAY_1 OFF");//Sending Events
  }
}

BLYNK_WRITE(V1)
{
  if (param.asInt() == 1) {
    digitalWrite(RELAY_PIN_2, HIGH);
    Blynk.logEvent("relay_state", "RELAY_2 ON");//Sending Events
  } else {
    digitalWrite(RELAY_PIN_2,  LOW);
    Blynk.logEvent("relay_state", "RELAY_2 OFF");//Sending Events
  }
}

BLYNK_WRITE(V2)
{
  if (param.asInt() == 1) {
    digitalWrite(RELAY_PIN_3, HIGH );
    Blynk.logEvent("relay_state", "RELAY_3 ON");//Sending Events
    delay(PUMP_DURATION_3);
    digitalWrite(RELAY_PIN_3, LOW );
    Blynk.logEvent("relay_state", "RELAY_3 OFF");//Sending Events
    Blynk.virtualWrite(V2, 0);
    Blynk.syncVirtual(V2);
  } else {
    digitalWrite(RELAY_PIN_3, LOW);
    Blynk.logEvent("relay_state", "RELAY_3 OFF");//Sending Events
  }
}

BLYNK_WRITE(V3)
{
  if (param.asInt() == 1) {
    digitalWrite(RELAY_PIN_4, HIGH );
    Blynk.logEvent("relay_state", "RELAY_4 ON");//Sending Events
    delay(PUMP_DURATION_4);
    digitalWrite(RELAY_PIN_4, LOW );
    Blynk.logEvent("relay_state", "RELAY_4 OFF");//Sending Events
    Blynk.virtualWrite(V3, 0);
    Blynk.syncVirtual(V3);
  } else {
    digitalWrite(RELAY_PIN_4, LOW);
    Blynk.logEvent("relay_state", "RELAY_4 OFF");//Sending Events
  }

  // if (param.asInt() == 1) {
  //   digitalWrite(RELAY_PIN_4, HIGH );
  //   Blynk.logEvent("relay_state", "RELAY_4 ON");//Sending Events
    
  //   if (water_level >= 3) {
  //     digitalWrite(RELAY_PIN_4, LOW );
  //     Blynk.logEvent("relay_state", "RELAY_4 OFF");//Sending Events
  //     Blynk.virtualWrite(V3, 0);
  //     Blynk.syncVirtual(V3);
  //   }
    
  // } else {
  //   digitalWrite(RELAY_PIN_4, LOW);
  //   Blynk.logEvent("relay_state", "RELAY_4 OFF");//Sending Events
  // }

}

BLYNK_WRITE(V4)
{
  PUMP_DURATION_4 = param.asInt();
}

BLYNK_WRITE(V5)
{
  digitalWrite(WATER_POWER_PIN, HIGH);  // turn the sensor ON
  delay(10);                      // wait 10 milliseconds
  water_value = analogRead(WATER_SENS_PIN); // read the analog value from sensor
  digitalWrite(WATER_POWER_PIN, LOW);   // turn the sensor OFF
  Blynk.virtualWrite(V5, water_value);

  delay(1000);
}

BLYNK_WRITE(V6)
{
  PUMP_DURATION_3 = param.asInt();
}

BLYNK_WRITE(V7)
{
  water_level = digitalRead(WATER_SWITCH_PIN); // read the digital value from sensor
  Blynk.virtualWrite(V7, water_level);

  delay(1000);
}

BLYNK_WRITE(V8)
{
  if (forecast == "Fair & Warm") {
    Blynk.virtualWrite(V7, 1);
  }

  delay(1000);
}

//Syncing the output state with the app at startup
BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V4);  // Pump duration 4
  // Blynk.syncVirtual(V5);  // water sens
  Blynk.syncVirtual(V6);  // Pump duration 3
  // Blynk.syncVirtual(V7);  // Display water level
  Blynk.syncVirtual(V0);  // Relay 1
  Blynk.syncVirtual(V1);  // Relay 2
  Blynk.syncVirtual(V2);  // Relay 3
  Blynk.syncVirtual(V3);  // Relay 4

  delay(10000);
  Blynk.virtualWrite(V7, 0);
  forecast = "NA"
}

void weatherAPI(){
  // Send an HTTP GET request to weather api
  client.println("GET /v1/environment/24-hour-weather-forecast");
  client.print("Host: ");
  client.println(host);
  client.println("Connection: close");  
  if (client.println() == 0) {
  Serial.println(F("Failed to send request"));
  client.stop();
  return;
  }

  //skip HTTP headers
  char endOfHeaders[] = "\r\n\r\n";
  if (!client.find(endOfHeaders)) {
    Serial.println(F("Invalid response"));
    client.stop();
    return;
  }

  // Parsing JSON file
  StaticJsonDocument<2048> doc;

  DeserializationError error = deserializeJson(doc, client);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  JsonObject items_0 = doc["items"][0];
  const char* items_0_update_timestamp = items_0["update_timestamp"]; // "2023-10-15T17:51:18+08:00"
  const char* items_0_timestamp = items_0["timestamp"]; // "2023-10-15T17:32:00+08:00"

  const char* items_0_valid_period_start = items_0["valid_period"]["start"];
  const char* items_0_valid_period_end = items_0["valid_period"]["end"]; // "2023-10-16T18:00:00+08:00"

  JsonObject items_0_general = items_0["general"];
  const char* items_0_general_forecast = items_0_general["forecast"]; // "Fair & Warm"

  int items_0_general_relative_humidity_low = items_0_general["relative_humidity"]["low"]; // 55
  int items_0_general_relative_humidity_high = items_0_general["relative_humidity"]["high"]; // 85

  int items_0_general_temperature_low = items_0_general["temperature"]["low"]; // 26
  int items_0_general_temperature_high = items_0_general["temperature"]["high"]; // 35

  int items_0_general_wind_speed_low = items_0_general["wind"]["speed"]["low"]; // 15
  int items_0_general_wind_speed_high = items_0_general["wind"]["speed"]["high"]; // 25

  const char* items_0_general_wind_direction = items_0_general["wind"]["direction"]; // "S"

  for (JsonObject items_0_period : items_0["periods"].as<JsonArray>()) {

    const char* items_0_period_time_start = items_0_period["time"]["start"]; // "2023-10-15T18:00:00+08:00", ...
    const char* items_0_period_time_end = items_0_period["time"]["end"]; // "2023-10-16T06:00:00+08:00", ...

    JsonObject items_0_period_regions = items_0_period["regions"];
    const char* items_0_period_regions_west = items_0_period_regions["west"]; // "Fair (Night)", "Fair & ...
    const char* items_0_period_regions_east = items_0_period_regions["east"]; // "Fair (Night)", "Fair & ...
    const char* items_0_period_regions_central = items_0_period_regions["central"]; // "Fair (Night)", "Fair ...
    const char* items_0_period_regions_south = items_0_period_regions["south"]; // "Fair (Night)", "Fair & ...
    const char* items_0_period_regions_north = items_0_period_regions["north"]; // "Fair (Night)", "Fair & ...

  }

  const char* api_info_status = doc["api_info"]["status"]; // "healthy"
}

void setup()
{
  // Debug console
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  //connecting to weather API
  Serial.print("Connecting to ");
  Serial.println(host);

  if (!client.connect(host, httpsPort)) {
    Serial.println("Connection failed");
    return;
  }

  pinMode(RELAY_PIN_1, OUTPUT);
  pinMode(RELAY_PIN_2, OUTPUT);
  pinMode(RELAY_PIN_3, OUTPUT);
  pinMode(RELAY_PIN_4, OUTPUT);
  
  pinMode(WATER_POWER_PIN, OUTPUT);   // configure D7 pin as an OUTPUT
  digitalWrite(WATER_POWER_PIN, LOW); // turn the sensor OFF

  delay(100);

  Blynk.begin(auth, ssid, pass);
}

void loop()
{
  weatherAPI();
  forecast = items_0_general_forecast;
  Blynk.run();

}