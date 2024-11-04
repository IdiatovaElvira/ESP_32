
#include <Arduino.h>
// #include <WiFiMulti.h>
#include <WiFi.h>
#include <Wire.h>

#define WIFI_SSID "wifi_name"   // wi-fi network name
#define WIFI_PASSWORD "-123456" // password

// WiFiMulti wifiMulti;

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  // while (wifiMulti.run() != WL_CONNECTED) {
  //   delay(100);
  // }

  Serial.println("Starting");
}

bool isConnected = false;

void loop() {
  if (WiFi.status() == WL_CONNECTED && !isConnected) {
    Serial.println("Connected");
    digitalWrite(LED_BUILTIN, HIGH);
    isConnected = true;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(".");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
    isConnected = false;
  }
  // digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
}