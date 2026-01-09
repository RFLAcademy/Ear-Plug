#include <Wire.h>
#include <Adafruit_DPS310.h>

#define VALVE_PIN 2   

#define VALVE_ON   HIGH
#define VALVE_OFF  LOW

#define DP_THRESHOLD 0.20   // hPa

#define ON_TIME     500     // ms
#define OFF_TIME    500     // ms
#define WAIT_TIME   10000   // ms

Adafruit_DPS310 dps1;
Adafruit_DPS310 dps2;

Adafruit_Sensor *p1;
Adafruit_Sensor *p2;

double base1 = 0, base2 = 0;

/* ============ VALVE STATE MACHINE ============ */
enum ValveState {
  IDLE,
  OPEN_PULSE,
  CLOSE_PULSE,
  WAIT_PHASE
};

ValveState valveState = IDLE;
unsigned long stateStartTime = 0;
uint8_t openCount = 0;
/* ============================================ */

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, VALVE_OFF);   // ✅ Valve OFF at boot

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

  double deltaP = (P1 - base1) - (P2 - base2);

  Serial.print("ΔP: ");
  Serial.println(deltaP, 6);

  unsigned long now = millis();

  /* ============ VALVE CONTROL LOGIC ============ */

  // Pressure OK → force valve OFF and reset
  if (abs(deltaP) <= DP_THRESHOLD) {
    digitalWrite(VALVE_PIN, VALVE_OFF);
    valveState = IDLE;
    openCount = 0;
    return;
  }

  switch (valveState) {

    case IDLE:
      digitalWrite(VALVE_PIN, VALVE_OFF);
      openCount = 0;
      valveState = OPEN_PULSE;
      stateStartTime = now;
      break;

    case OPEN_PULSE:
      digitalWrite(VALVE_PIN, VALVE_ON);   // 🔓 OPEN valve
      if (now - stateStartTime >= ON_TIME) {
        digitalWrite(VALVE_PIN, VALVE_OFF);
        valveState = CLOSE_PULSE;
        stateStartTime = now;
      }
      break;

    case CLOSE_PULSE:
      digitalWrite(VALVE_PIN, VALVE_OFF);
      if (now - stateStartTime >= OFF_TIME) {
        openCount++;
        if (openCount >= 2) {
          valveState = WAIT_PHASE;
        } else {
          valveState = OPEN_PULSE;
        }
        stateStartTime = now;
      }
      break;

    case WAIT_PHASE:
      digitalWrite(VALVE_PIN, VALVE_OFF);
      if (now - stateStartTime >= WAIT_TIME) {
        valveState = IDLE;
      }
      break;
  }
}