// src/main.cpp - ESP32 Mission Planner Web Server + UART + Live Drone Status

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <HardwareSerial.h>
#include <SPIFFS.h>

#define WIFI_SSID     "Sss"
#define WIFI_PASS     "KiChaoBro**"
#define UART_BAUD     57600
#define UART_TX       17  // ESP32 TX to STM32 RX
#define UART_RX       16  // ESP32 RX from STM32 TX

AsyncWebServer server(80);
HardwareSerial SerialUART(2); // Use UART2

String latestStatusJSON = "{}";
String incomingUART = "";
unsigned long lastStatusUpdate = 0;

// Fallback mock for debugging
String mockStatusJSON() {
  return R"({
    "status": "Waiting for STM32",
    "gps": 0,
    "lat": 0.0,
    "lon": 0.0,
    "alt": 0,
    "ultrasonic": 0
  })";
}

void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Mission Planner");

  SerialUART.begin(UART_BAUD, SERIAL_8N1, UART_RX, UART_TX);

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.println("WiFi AP started");
  Serial.println(WiFi.softAPIP());

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS failed to mount");
    return;
  }

  if (!SPIFFS.exists("/index.html")) {
    Serial.println("index.html NOT FOUND in SPIFFS!");
  } else {
    Serial.println("index.html FOUND in SPIFFS!");
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (latestStatusJSON.length() > 5) {
      request->send(200, "application/json", latestStatusJSON);
    } else {
      request->send(200, "application/json", mockStatusJSON());
    }
  });

  server.on("/mission", HTTP_POST, [](AsyncWebServerRequest *request){}, NULL,
    [](AsyncWebServerRequest *request, uint8_t* data, size_t len, size_t index, size_t total) {
      Serial.println("Received Mission:");
      for (size_t i = 0; i < len; i++) Serial.write(data[i]);
      Serial.println();

      // Forward the mission to STM32 via UART
      SerialUART.write(data, len);
      request->send(200, "text/plain", "Mission received");
    });

  server.onNotFound(notFound);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  // Read incoming UART from STM32
  while (SerialUART.available()) {
    char c = SerialUART.read();
    if (c == '\n') {
      latestStatusJSON = incomingUART;
      incomingUART = "";
      lastStatusUpdate = millis();
    } else {
      incomingUART += c;
    }
  }

  // Optional: reset stale status after timeout (10s)
  if (millis() - lastStatusUpdate > 10000) {
    latestStatusJSON = mockStatusJSON();
  }
} 
