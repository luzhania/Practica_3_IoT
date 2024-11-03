#include <WiFi.h>
#pragma once

class WiFiConnection {
private:
  const char* SSID;
  const char* PASSWORD;

public:
  WiFiConnection(const char* SSID, const char* PASSWORD)
      : SSID(SSID), PASSWORD(PASSWORD) {}

  void connect() {
    WiFi.begin(SSID, PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
      delay(1000);
      Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
  }
};