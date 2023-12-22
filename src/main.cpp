#include <Arduino.h>

const int analogPin = A0;
const int relayPin = 7;

const int bufferSize = 5;
float voltageBuffer[bufferSize];

float voltageThresholdHigh = 2.20;
float voltageThresholdLow = 2.00;
bool relayState = LOW;

float analogToVoltage(int analogValue);
float calculateMovingAverage();
float removeOutliers(float value);

void setup() {
  Serial.begin(9600);
  pinMode(relayPin, OUTPUT);
  analogReference(INTERNAL);

  for (int i = 0; i < bufferSize; i++) {
    voltageBuffer[i] = analogToVoltage(analogRead(analogPin));
  }
}

void loop() {
  for (int i = bufferSize - 1; i > 0; i--) {
    voltageBuffer[i] = voltageBuffer[i - 1];
  }

  voltageBuffer[0] = analogToVoltage(analogRead(analogPin));

  float movingAverage = calculateMovingAverage();
  float filteredVoltage = removeOutliers(movingAverage);

  if (filteredVoltage <= voltageThresholdLow && relayState == HIGH) {
    digitalWrite(relayPin, LOW);
    relayState = LOW;  
  } else if (filteredVoltage >= voltageThresholdHigh && relayState == LOW) {
    digitalWrite(relayPin, HIGH);
    relayState = HIGH;  
  }

  Serial.print("Voltage: ");
  Serial.print(filteredVoltage, 2);
  Serial.println(" V");
  Serial.print("Relay State: ");
  Serial.println(relayState);
  

  delay(1000);
}

float analogToVoltage(int analogValue) {
  return analogValue * (1.1 / 1023.0) * (3.0 / 0.94) - (analogValue * (1.1 / 1023.0) * 0.03);
}

float calculateMovingAverage() {
  float sum = 0.0;
  for (int i = 0; i < bufferSize; i++) {
    sum += voltageBuffer[i];
  }
  return sum / bufferSize;
}

float removeOutliers(float value) {
  const float outlierThreshold = 0.2;

  if (abs(value - calculateMovingAverage()) > outlierThreshold) {
    return calculateMovingAverage();
  } else {
    return value;
  }
}
