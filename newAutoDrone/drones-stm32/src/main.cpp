#include <Arduino.h>
#include <Servo.h>

Servo esc1, esc2, esc3, esc4;

void setup() {
  Serial.begin(57600);
  Serial.println("ESC Test Starting...");

  esc1.attach(PA0);
  esc2.attach(PA1);
  esc3.attach(PA2);
  esc4.attach(PA3);

  // Minimum signal to initialize ESCs
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);

  Serial.println("ESC armed at 1000us (min throttle)");
  delay(5000); // WAIT for 5 seconds to complete initialization

  // Gently ramp up
  Serial.println("Increasing throttle to 1200...");
  esc1.writeMicroseconds(1000);
  esc2.writeMicroseconds(1000);
  esc3.writeMicroseconds(1000);
  esc4.writeMicroseconds(1000);
}

void loop() {
  // Do nothing
}
