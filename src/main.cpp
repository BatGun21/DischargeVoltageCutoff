#include <Arduino.h>

const int analogPin = A0;
const int relayPin = 7;

float voltageThresholdHigh = 2.35;
float voltageThresholdLow = 2.28;  
bool relayState = HIGH;  // Initial state (assumes relay is initially on)

void setup() {
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  analogReference(INTERNAL);
}

void loop() {
  int sensorValue = analogRead(analogPin);
  float voltage = sensorValue * (1.1 / 1023.0);
  voltage = voltage * (3.0/ 0.94); // Correction for voltage divider
  voltage -= voltage * 0.04; // Correction using observation

  if (voltage <= voltageThresholdLow && relayState == HIGH) {
    digitalWrite(relayPin, LOW);
    Serial.println("Relay OFF");
    relayState = LOW;  
  } else if (voltage >= voltageThresholdHigh && relayState == LOW) {
    digitalWrite(relayPin, HIGH);
    Serial.println("Relay ON");
    relayState = HIGH;  
  }

  Serial.print("Voltage: ");
  Serial.print(voltage, 2); // Print with 2 decimal places
  Serial.println(" V");

  delay(1000);
}
