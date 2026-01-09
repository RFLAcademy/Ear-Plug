#include <Wire.h>
#include <Adafruit_DPS310.h>

#define RELAY_PIN 27
#define DP_THRESHOLD 0.20        // hPa
#define TOGGLE_INTERVAL 500      // ms

Adafruit_DPS310 dps1;
Adafruit_DPS310 dps2;

Adafruit_Sensor *p1;
Adafruit_Sensor *p2;

double base1 = 0, base2 = 0;

bool relayState = LOW;
unsigned long lastToggleTime = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);   // Relay OFF initially

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!dps1.begin_I2C(0x77)) {
    Serial.println("❌ DPS310 #1 not found");
    while (1);
  }

  if (!dps2.begin_I2C(0x76)) {
    Serial.println("❌ DPS310 #2 not found");
    while (1);
  }

  Serial.println("✅ Both DPS310 sensors detected");

  dps1.configurePressure(DPS310_64HZ, DPS310_128SAMPLES);
  dps2.configurePressure(DPS310_64HZ, DPS310_128SAMPLES);

  p1 = dps1.getPressureSensor();
  p2 = dps2.getPressureSensor();

  Serial.println("📌 Calibrating baseline...");
  delay(3000);
}

void loop() {
  sensors_event_t e1, e2;
  p1->getEvent(&e1);
  p2->getEvent(&e2);

  double P1 = e1.pressure;
  double P2 = e2.pressure;

  // Set baseline once
  if (base1 == 0 && base2 == 0) {
    base1 = P1;
    base2 = P2;
    Serial.println("✅ Baselines set");
    return;
  }

  double deltaP = P1 - P2;

  Serial.print("P1: "); Serial.print(P1, 4);
  Serial.print(" | P2: "); Serial.print(P2, 4);
  Serial.print(" | ΔP: "); Serial.println(deltaP, 6);

  unsigned long now = millis();

  // Check threshold
  if (deltaP > DP_THRESHOLD || deltaP < -DP_THRESHOLD) {

    if (now - lastToggleTime >= TOGGLE_INTERVAL) {
      relayState = !relayState;
      digitalWrite(RELAY_PIN, relayState);
      lastToggleTime = now;
    }

  } else {
    // Inside deadband → stop relay
    relayState = LOW;
    digitalWrite(RELAY_PIN, LOW);
  }
}