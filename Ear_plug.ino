#include "HX711.h"

#define DT1 D3
#define SCK1 D2
#define DT2 D5
#define SCK2 D4

#define PUMP_SUCK D7
#define PUMP_INJECT D8

HX711 pressureSensor1;
HX711 pressureSensor2;

#define NUM_SAMPLES 10
#define ALPHA 0.05   // EMA smoothing
#define DRIFT_LIMIT 0.0005
#define PRESSURE_TOLERANCE 0.3  // atm

long referencePressure1 = 0;
long referencePressure2 = 0;
float filteredPressure1 = 0;
float filteredPressure2 = 0;

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_SUCK, OUTPUT);
  pinMode(PUMP_INJECT, OUTPUT);
  digitalWrite(PUMP_SUCK, LOW);
  digitalWrite(PUMP_INJECT, LOW);

  pressureSensor1.begin(DT1, SCK1);
  pressureSensor2.begin(DT2, SCK2);

  delay(1000);
  Serial.println("Calibrating both sensors... Taking 10 readings each for reference (1 atm).");

  long sum1 = 0;
  long sum2 = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum1 += pressureSensor1.read();
    sum2 += pressureSensor2.read();
    delay(200);
  }

  referencePressure1 = sum1 / NUM_SAMPLES;
  referencePressure2 = sum2 / NUM_SAMPLES;

  filteredPressure1 = referencePressure1;
  filteredPressure2 = referencePressure2;

  Serial.println("-----------------------------------");
  Serial.print("✅ Reference 1 (1 atm): "); Serial.println(referencePressure1);
  Serial.print("✅ Reference 2 (1 atm): "); Serial.println(referencePressure2);
  Serial.println("-----------------------------------");
}

void loop() {
  long raw1 = pressureSensor1.read();
  long raw2 = pressureSensor2.read();

  // EMA smoothing
  filteredPressure1 = (ALPHA * raw1) + ((1 - ALPHA) * filteredPressure1);
  filteredPressure2 = (ALPHA * raw2) + ((1 - ALPHA) * filteredPressure2);

  long diff1 = referencePressure1 - (long)filteredPressure1;
  long diff2 = referencePressure2 - (long)filteredPressure2;

  float atm1 = 1.0 + ((float)diff1 / referencePressure1);
  float atm2 = 1.0 + ((float)diff2 / referencePressure2);

  // Drift correction
  if (fabs(atm1 - 1.0) < DRIFT_LIMIT) atm1 = 1.0;
  if (fabs(atm2 - 1.0) < DRIFT_LIMIT) atm2 = 1.0;

  // Pressure balancing logic
  float pressureDiff = atm1 - atm2;

  if (fabs(pressureDiff) <= PRESSURE_TOLERANCE) {
    // No balancing needed
    digitalWrite(PUMP_SUCK, LOW);
    digitalWrite(PUMP_INJECT, LOW);
  } else if (pressureDiff > PRESSURE_TOLERANCE) {
    // Outside pressure higher → inject air inside
    digitalWrite(PUMP_INJECT, HIGH);
    digitalWrite(PUMP_SUCK, LOW);
  } else if (pressureDiff < -PRESSURE_TOLERANCE) {
    // Inside pressure higher → suck air out
    digitalWrite(PUMP_INJECT, LOW);
    digitalWrite(PUMP_SUCK, HIGH);
  }

  Serial.print("Sensor1 -> Atm: "); Serial.print(atm1, 6);
  Serial.print("  |  Sensor2 -> Atm: "); Serial.print(atm2, 6);
  Serial.print("  |  ΔP: "); Serial.println(pressureDiff, 6);

  delay(150);
}
