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

    // Handle browser icon requests silently
    server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(204);
    });

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
