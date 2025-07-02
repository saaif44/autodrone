<<<<<<< HEAD
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WebSocketsServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// --- Configuration ---
const char* WIFI_SSID = "DRONE_WIFI";
const char* WIFI_PASS = "pass1234";

// UART for STM32 communication
#define STM_SERIAL Serial1
#define STM_TX_PIN 17
#define STM_RX_PIN 18
#define STM_BAUD_RATE 57600

// --- Global Objects ---
AsyncWebServer server(80);
WebSocketsServer webSocket(81);

// --- Function Prototypes ---
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
    Serial.begin(115200);
    Serial.println("\nESP32 Ground Control Booting...");

    STM_SERIAL.begin(STM_BAUD_RATE, SERIAL_8N1, STM_TX_PIN, STM_RX_PIN);
    Serial.println("STM32 UART Link Initialized.");

    if (!LittleFS.begin()) {
        Serial.println("FATAL: LittleFS mount failed.");
        return;
    }

    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    // --- Web Server Routes ---
    // Redirect root to the health check page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("/health.html");
    });

    // Serve static files from LittleFS
    server.serveStatic("/", LittleFS, "/");

    // --- WebSocket Setup ---
    webSocket.begin();
    webSocket.onEvent(onWebSocketEvent);

    server.begin();
    Serial.println("Web Server & WebSocket Server started.");
    Serial.println("Connect to DRONE_WIFI and navigate to http://192.168.4.1");
}

void loop() {
    webSocket.loop(); // Must be called to process WebSocket events

    // Check for incoming telemetry from STM32 and broadcast it
    if (STM_SERIAL.available()) {
        String stmMessage = STM_SERIAL.readStringUntil('\n');
        stmMessage.trim();
        if (stmMessage.length() > 0) {
            // Forward the raw JSON from STM32 to all connected web clients
            webSocket.broadcastTXT(stmMessage);
            Serial.print("STM32 -> BCAST: ");
            Serial.println(stmMessage);
        }
    }
}

/**
 * @brief Handles incoming WebSocket messages from the web browser.
 */
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED: {
            IPAddress ip = webSocket.remoteIP(num);
            Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
            break;
        }
        case WStype_TEXT: {
            Serial.printf("[%u] Received Text: %s\n", num, payload);
            
            // The payload is the JSON string from the browser.
            // Forward it directly to the STM32.
            STM_SERIAL.println((char*)payload);
            
            // Optional: send an ack back to the client
            // webSocket.sendTXT(num, "Command received!");
            break;
        }
        default:
            break;
    }
}
=======
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
>>>>>>> 667434c7b5a26e30f5ebfe183126244b8b6e938f
