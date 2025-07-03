#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS++.h>
#include <ArduinoJson.h>

// --- Pinout Configuration (Matches your diagram) ---
// Communication with ESP32
#define ESP_SERIAL Serial1 // PA9 (TX), PA10 (RX)
#define ESP_BAUD_RATE 57600

// Communication with GPS Module
#define GPS_SERIAL Serial2 // PA2 (TX), PA3 (RX)
#define GPS_BAUD_RATE 9600

// MPU6050 I2C (Default pins for STM32)
// SCL: PB6, SDA: PB7

// ESC Motor Pins
#define MOTOR1_PIN PA0
#define MOTOR2_PIN PA1
#define MOTOR3_PIN PA2
#define MOTOR4_PIN PA3

// Battery Voltage Sensing Pin
#define VBAT_PIN PA4
const float VOLTAGE_DIVIDER_RATIO = 11.0; // Adjust this to match your voltage divider (R1+R2)/R2

// --- Global Objects ---
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
Servo motor1, motor2, motor3, motor4;

// --- Drone State & Telemetry ---
enum DroneState {
    STATE_IDLE,
    STATE_ARMED,
    STATE_VIBE_TEST,
    STATE_MISSION,
    STATE_FAILSAFE
};
DroneState currentState = STATE_IDLE;

struct TelemetryData {
    bool mpu_ok = false;
    double lat = 0.0, lon = 0.0, alt = 0.0, heading = 0.0;
    uint32_t sats = 0;
    float batteryV = 0.0;
    float vibration_level = 0.0;
} telemetry;

// --- Timers ---
unsigned long lastTelemetryTime = 0;
const long telemetryInterval = 500; // Send telemetry every 500ms

// --- Forward Declarations ---
void handleEsp32Communication();
void readSensors();
void sendTelemetry();
void parseMissionCommand(JsonDocument& doc);
void startVibrationTest();

// =================================================================
//   SETUP
// =================================================================
void setup() {
    // Start debug serial
    Serial.begin(115200);
    Serial.println("STM32 Flight Controller Booting...");

    // Initialize I2C for MPU6050
    Wire.begin();

    // Initialize MPU6050
    if (!mpu.begin()) {
        Serial.println("FATAL: Failed to find MPU6050 chip");
        telemetry.mpu_ok = false;
        currentState = STATE_FAILSAFE;
    } else {
        Serial.println("MPU6050 Found!");
        telemetry.mpu_ok = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }

    // Initialize UARTs
    ESP_SERIAL.begin(ESP_BAUD_RATE);
    GPS_SERIAL.begin(GPS_BAUD_RATE);
    Serial.println("UARTs Initialized.");

    // Attach ESCs
    // NOTE: For real flight, you NEED an ESC calibration sequence here.
    motor1.attach(MOTOR1_PIN, 1000, 2000); // (pin, min pulse, max pulse)
    motor2.attach(MOTOR2_PIN, 1000, 2000);
    motor3.attach(MOTOR3_PIN, 1000, 2000);
    motor4.attach(MOTOR4_PIN, 1000, 2000);
    
    // Disarm motors by sending min throttle signal
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);
    Serial.println("ESCs Attached & Disarmed.");

    // Set analog pin for battery
    pinMode(VBAT_PIN, INPUT_ANALOG);

    Serial.println("Setup Complete. Entering main loop.");
}

// =================================================================
//   MAIN LOOP
// =================================================================
void loop() {
    // Always listen for commands and GPS data
    handleEsp32Communication();
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // Read all sensor values
    readSensors();

    // Main state machine
    switch(currentState) {
        case STATE_IDLE:
            // Do nothing, wait for commands
            break;
        case STATE_VIBE_TEST:
            // This state is managed by the startVibrationTest function
            // It will automatically transition back to IDLE
            break;
        case STATE_ARMED:
            // *** PID FLIGHT CONTROL LOGIC GOES HERE ***
            // This is where you would calculate roll, pitch, yaw errors
            // and adjust motor speeds to stabilize the drone.
            break;
        case STATE_MISSION:
            // *** MISSION EXECUTION LOGIC GOES HERE ***
            // Process the current waypoint, navigate, and move to the next.
            break;
        case STATE_FAILSAFE:
            // Critical error. Cut motors.
            motor1.writeMicroseconds(1000);
            motor2.writeMicroseconds(1000);
            motor3.writeMicroseconds(1000);
            motor4.writeMicroseconds(1000);
            break;
    }

    // Periodically send telemetry data back to the ESP32
    if (millis() - lastTelemetryTime > telemetryInterval) {
        sendTelemetry();
        lastTelemetryTime = millis();
    }
}

// =================================================================
//   CORE FUNCTIONS
// =================================================================

/**
 * @brief Reads all sensor data and updates the global telemetry struct.
 */
void readSensors() {
    // Read MPU
    sensors_event_t a, g, temp;
    if (telemetry.mpu_ok) {
        mpu.getEvent(&a, &g, &temp);
        // Simple vibration check: sum of squares of acceleration changes
        // A more robust method would use a high-pass filter.
        telemetry.vibration_level = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z - 9.8, 2));
    }

    // Read GPS
    if (gps.location.isValid()) {
        telemetry.lat = gps.location.lat();
        telemetry.lon = gps.location.lng();
        telemetry.alt = gps.altitude.meters();
        telemetry.sats = gps.satellites.value();
        telemetry.heading = gps.course.deg();
    }

    // Read Battery Voltage
    int raw_vbat = analogRead(VBAT_PIN);
    float voltage = (raw_vbat / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
    telemetry.batteryV = voltage;
}

/**
 * @brief Listens for JSON commands from ESP32 and processes them.
 */
void handleEsp32Communication() {
    if (ESP_SERIAL.available()) {
        String input = ESP_SERIAL.readStringUntil('\n');
        
        StaticJsonDocument<1024> doc; // Adjust size for mission plan
        DeserializationError error = deserializeJson(doc, input);

        if (error) {
            Serial.print("JSON deserialize failed: ");
            Serial.println(error.c_str());
            return;
        }

        // Check for specific commands
        if (doc.containsKey("command")) {
            const char* command = doc["command"];
            if (strcmp(command, "start_vibration_test") == 0) {
                startVibrationTest();
            }
        } 
        // Check for a mission plan
        else if (doc.containsKey("mission")) {
            parseMissionCommand(doc);
        }
    }
}

/**
 * @brief Serializes the telemetry struct to JSON and sends it to the ESP32.
 */
void sendTelemetry() {
    StaticJsonDocument<256> doc;

    doc["type"] = "telemetry";
    doc["lat"] = telemetry.lat;
    doc["lon"] = telemetry.lon;
    doc["sats"] = telemetry.sats;
    doc["batteryV"] = round(telemetry.batteryV * 100) / 100.0; // 2 decimal places
    doc["mpu_ok"] = telemetry.mpu_ok;
    doc["vibration_level"] = telemetry.vibration_level;
    
    // Add state as a string
    switch(currentState) {
        case STATE_IDLE: doc["state"] = "IDLE"; break;
        case STATE_VIBE_TEST: doc["state"] = "VIBE_TEST"; break;
        case STATE_ARMED: doc["state"] = "ARMED"; break;
        case STATE_MISSION: doc["state"] = "MISSION"; break;
        case STATE_FAILSAFE: doc["state"] = "FAILSAFE"; break;
    }

    String output;
    serializeJson(doc, output);
    ESP_SERIAL.println(output);
}

/**
 * @brief Parses a mission plan from the ESP32 and stores it.
 * @param doc The parsed JSON document containing the mission.
 */
void parseMissionCommand(JsonDocument& doc) {
    Serial.println("Received mission plan from ESP32.");
    
    // NOTE: This is where you would store the mission waypoints into an array or list.
    // For this example, we just print the contents.
    JsonArray missionSteps = doc["mission"];
    bool returnHome = doc["return_home"];

    Serial.printf("Mission has %d steps. Return to Home: %s\n", missionSteps.size(), returnHome ? "Yes" : "No");

    for(JsonObject step : missionSteps) {
        const char* action = step["action"];
        int alt = step["alt"];
        Serial.printf("- Action: %s, Alt: %d\n", action, alt);
    }
    
    // Once parsed, you would set the state to start the mission
    // currentState = STATE_MISSION; 
    // mission_waypoint_index = 0;
}


/**
 * @brief Runs the motors at a low throttle to test for vibrations.
 */
void startVibrationTest() {
    Serial.println("Starting Vibration Test...");
    currentState = STATE_VIBE_TEST;

    // Send telemetry update immediately to show "Testing..." on web UI
    sendTelemetry();

    int testThrottle = 1150; // Low throttle, just above spin-up
    motor1.writeMicroseconds(testThrottle);
    motor2.writeMicroseconds(testThrottle);
    motor3.writeMicroseconds(testThrottle);
    motor4.writeMicroseconds(testThrottle);

    // Run test for 5 seconds
    delay(5000);

    // Disarm motors
    motor1.writeMicroseconds(1000);
    motor2.writeMicroseconds(1000);
    motor3.writeMicroseconds(1000);
    motor4.writeMicroseconds(1000);

    Serial.println("Vibration Test Complete.");
    currentState = STATE_IDLE;

    // Send final telemetry so web UI can update with pass/fail
    readSensors(); // Get one last reading
    sendTelemetry();
}