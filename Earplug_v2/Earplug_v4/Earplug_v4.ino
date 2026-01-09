#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_DPS310.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <EEPROM.h>

/* ================== VALVE CONFIG ================== */
#define VALVE_PIN 2
#define VALVE_ON   HIGH
#define VALVE_OFF  LOW

#define DP_THRESHOLD 0.05
#define ON_TIME   500
#define OFF_TIME  500

volatile unsigned long waitTimeMs = 10000;

/* ================== DPS310 ================== */
Adafruit_DPS310 dps1, dps2;
Adafruit_Sensor *p1, *p2;

double base1 = 0, base2 = 0;
double P1 = 0, P2 = 0, deltaP = 0;

bool baselineSet = false;   // ✅ NEW

/* ================== VALVE FSM ================== */
enum ValveState { IDLE, OPEN_PULSE, CLOSE_PULSE, WAIT_PHASE };
ValveState valveState = IDLE;
unsigned long stateStartTime = 0;
uint8_t openCount = 0;

bool pressureTriggered = false;

/* ================== WIFI ================== */
const char* ssid = "PressureControlESP";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncEventSource events("/events");

/* ================== CSV LOGGING ================== */
const char* CSV_PATH = "/pressure_log.csv";

unsigned long bootMillis = 0;
unsigned long lastCSVWrite = 0;

/* ================== STATE STRING ================== */
const char* valveStateToString(ValveState s) {
  switch (s) {
    case IDLE:        return "IDLE";
    case OPEN_PULSE:  return "OPEN_PULSE";
    case CLOSE_PULSE: return "CLOSE_PULSE";
    case WAIT_PHASE:  return "WAIT_PHASE";
    default:          return "UNKNOWN";
  }
}

/* ================== EEPROM ================== */
#define EEPROM_SIZE 4
#define EEPROM_ADDR_DELAY 0

/* ================== HTML ================== */
const char index_html[] PROGMEM =
"<!DOCTYPE html><html><head>"
"<meta charset='utf-8'>"
"<title>Adaptive Air Pressure Regulator</title>"
"<style>"
"body{background:black;color:#39ff14;font-family:Arial;text-align:center;"
"text-shadow:0 0 10px #39ff14;}"
"canvas{border:1px solid #39ff14;margin-top:20px;}"
".pressure-row{display:flex;justify-content:space-around;margin-top:15px;font-size:18px;}"
".pressure-box{width:30%;}"
"input[type=range]{width:70%;}"
"button{background:black;color:#39ff14;border:2px solid #39ff14;"
"padding:10px 25px;box-shadow:0 0 10px #39ff14;cursor:pointer;}"
"#status{margin-top:10px;color:#39ff14;font-size:16px;}"
".slider-row{display:flex;align-items:center;justify-content:center;gap:20px;}"

".slider-wrap{position:relative;width:70%;}"

"input[type=range]{width:100%;}"

".bubble{position:absolute;top:-35px;left:50%;transform:translateX(-50%);background:black;border:1px solid #39ff14;padding:4px 8px;font-size:14px;box-shadow:0 0 8px #39ff14;pointer-events:none; display:none;}"

"</style></head><body>"

"<h1>Adaptive Air Pressure Regulator</h1>"

"<canvas id='graph' width='900' height='450'></canvas>"

"<div class='pressure-row'>"
"<div class='pressure-box'>Outside<br><span id='p1'>--</span> hPa</div>"
"<div class='pressure-box'>ΔP<br><span id='dp'>--</span> hPa</div>"
"<div class='pressure-box'>Inside<br><span id='p2'>--</span> hPa</div>"
"</div>"
"<br>"
"<br>"
"<div class='slider-row'>"
  "<div class='slider-wrap'>"
    "<input type='range' min='5' max='15' value='10' id='slider'>"
    "<div id='bubble' class='bubble'></div>"
  "</div>"

  "<button onclick='setTime()'>SET</button>"
"</div>"

"<div id='connStatus'>Waiting for data...</div>"
"<div id='setStatus'></div>"
"<br><br>"
"<button onclick='downloadCSV()'>DOWNLOAD CSV</button>"


"<script>"
"var curDelay = document.getElementById('curDelay');"
"var canvas=document.getElementById('graph');"
"var ctx=canvas.getContext('2d');"

"var p1El=document.getElementById('p1');"
"var p2El=document.getElementById('p2');"
"var dpEl=document.getElementById('dp');"

"var slider=document.getElementById('slider');"
"var sval=document.getElementById('sval');"
"var connStatus=document.getElementById('connStatus');"
"var setStatus=document.getElementById('setStatus');"

"var outData=[];"
"var inData=[];"

"var Y_MIN=1010;"
"var Y_MAX=1100;"

"function mapY(v){return 430-((v-Y_MIN)/(Y_MAX-Y_MIN))*420;}"

"function downloadCSV() {"
"  window.location.href = '/download';"
"}"

"function drawGraph(){"
"ctx.clearRect(0,0,900,450);"

"ctx.strokeStyle='#39ff14';ctx.beginPath();ctx.moveTo(80,10);ctx.lineTo(80,430);ctx.stroke();"

"ctx.fillStyle='#39ff14';ctx.font='12px Arial';"
"for(var p=1010;p<=1100;p+=5){"
"var y=mapY(p);"
"ctx.fillText(p+' hPa',2,y+4);"
"ctx.strokeStyle='#003300';ctx.beginPath();ctx.moveTo(80,y);ctx.lineTo(900,y);ctx.stroke();"
"}"

"if(outData.length<2)return;"

"ctx.strokeStyle='#39ff14';ctx.beginPath();"
"for(var i=0;i<outData.length;i++){"
"var x=80+i*(860/(outData.length-1));"
"var y=mapY(outData[i]);"
"if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);"
"}ctx.stroke();"

"ctx.strokeStyle='#00ffaa';ctx.beginPath();"
"for(var i=0;i<inData.length;i++){"
"var x=80+i*(860/(inData.length-1));"
"var y=mapY(inData[i]);"
"if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);"
"}ctx.stroke();"
"}"

"var evt=new EventSource('/events');"

"evt.onopen=function(){"
"connStatus.innerHTML='✔ Connected to ESP32';"
"};"

"evt.onerror=function(){"
"connStatus.innerHTML='Connection lost, retrying...';"
"};"

"evt.onmessage=function(e){"
"var d=JSON.parse(e.data);"
"p1El.innerHTML=d.p1.toFixed(2);"
"p2El.innerHTML=d.p2.toFixed(2);"
"dpEl.innerHTML=d.dp.toFixed(3);"
"outData.push(d.p1);"
"inData.push(d.p2);"
"if(outData.length>50){outData.shift();inData.shift();}"
"drawGraph();"
"};"

"var bubble = document.getElementById('bubble');"

"function updateBubble(){"
  "var val = slider.value;"
  "var min = slider.min;"
  "var max = slider.max;"

  "var percent = (val - min) / (max - min);"

  "bubble.innerHTML = val;"
  "bubble.style.left = (percent * 100) + '%';"
"}"

"slider.addEventListener('input', function () {"
  "bubble.style.display = 'block';"
  "updateBubble();"
"});" 

"function setTime(){"
  "fetch('/set?time='+slider.value)"
    ".then(() => {"
      "setStatus.innerHTML='✔ Delay set to '+slider.value;"
      "curDelay.innerHTML = slider.value;"
      "updateBubble()"
    "});"
"}"

"window.onload = function () {"
  "fetch('/delay')"
    ".then(r => r.text())"
    ".then(v => {"
      "slider.value = v;"
      "curDelay.innerHTML = v;"
      "requestAnimationFrame(updateBubble);"
    "});"
"};"
"</script></body></html>";

String formatTime(unsigned long seconds) {
  unsigned long h = seconds / 3600;
  unsigned long m = (seconds % 3600) / 60;
  unsigned long s = seconds % 60;

  char buf[12];
  snprintf(buf, sizeof(buf), "%02lu:%02lu:%02lu", h, m, s);
  return String(buf);
}

/* ================== SETUP ================== */
void setup() {
  Serial.begin(115200);
  /* ===== SPIFFS INIT ===== */
  if (!SPIFFS.begin(true)) {
    Serial.println("❌ SPIFFS Mount Failed");
  } else {
    Serial.println("✅ SPIFFS Mounted");

    // Create new CSV on every boot
    File f = SPIFFS.open(CSV_PATH, FILE_WRITE);
    if (f) {
      f.println("time,outside_pressure,inside_pressure,pressure_difference,set_delay");
      f.close();
      Serial.println("📄 New CSV file created");
    }
  }

  bootMillis = millis();

  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, VALVE_OFF);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  dps1.begin_I2C(0x77);
  dps2.begin_I2C(0x76);

  dps1.configurePressure(DPS310_64HZ, DPS310_128SAMPLES);
  dps2.configurePressure(DPS310_64HZ, DPS310_128SAMPLES);

  p1 = dps1.getPressureSensor();
  p2 = dps2.getPressureSensor();

  Serial.println("📌 Calibrating baseline...");
  delay(3000);   // same intent as your reference code

  /* ===== EEPROM INIT ===== */
  EEPROM.begin(EEPROM_SIZE);

  uint32_t storedDelay = EEPROM.readUInt(EEPROM_ADDR_DELAY);
  if (storedDelay >= 5000 && storedDelay <= 15000) {
    waitTimeMs = storedDelay;
    Serial.print("📦 Loaded delay from EEPROM: ");
    Serial.println(waitTimeMs);
  } else {
    waitTimeMs = 10000;  // default 10s
    EEPROM.writeUInt(EEPROM_ADDR_DELAY, waitTimeMs);
    EEPROM.commit();
    Serial.println("🆕 EEPROM initialized with default delay");
  }

  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){
    r->send_P(200, "text/html", index_html);
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *r){
    if (r->hasParam("time")) {
      waitTimeMs = r->getParam("time")->value().toInt() * 1000UL;

      EEPROM.writeUInt(EEPROM_ADDR_DELAY, waitTimeMs);
      EEPROM.commit();

      Serial.print("💾 Delay saved to EEPROM: ");
      Serial.println(waitTimeMs);
    }
    r->send(200, "text/plain", "OK");
  });


  server.on("/download", HTTP_GET, [](AsyncWebServerRequest *request){
    if (SPIFFS.exists(CSV_PATH)) {
      request->send(SPIFFS, CSV_PATH, "text/csv", true);
    } else {
      request->send(404, "text/plain", "CSV not found");
    }
  });

  server.on("/delay", HTTP_GET, [](AsyncWebServerRequest *r){
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", waitTimeMs / 1000);
    r->send(200, "text/plain", buf);
  });



  server.addHandler(&events);
  server.begin();
}

/* ================== LOOP ================== */
void loop() {
  sensors_event_t e1, e2;
  p1->getEvent(&e1);
  p2->getEvent(&e2);

  P1 = e1.pressure;
  P2 = e2.pressure;

  /* ===== BASELINE CALIBRATION (ONCE) ===== */
  if (!baselineSet) {
    base1 = P1;
    base2 = P2;
    baselineSet = true;

    Serial.println("✅ Baseline calibrated");
    Serial.print("Base1: "); Serial.println(base1);
    Serial.print("Base2: "); Serial.println(base2);

    digitalWrite(VALVE_PIN, VALVE_OFF);
    return;
  }

  /* ===== DIFFERENTIAL PRESSURE ===== */
  deltaP = (P1 - base1) - (P2 - base2);

  /* ===== CSV LOGGING (1 Hz) ===== */
  if (millis() - lastCSVWrite >= 1000) {
    lastCSVWrite = millis();

    unsigned long elapsedSec = (millis() - bootMillis) / 1000;
    String timeStr = formatTime(elapsedSec);

    File f = SPIFFS.open(CSV_PATH, FILE_APPEND);
    if (f) {
      f.printf("%s,%.2f,%.2f,%.4f,%lu\n",
        timeStr.c_str(),
        P1,
        P2,
        deltaP,
        waitTimeMs / 1000
      );
      f.close();
    }
  }

  static unsigned long lastSend = 0;
  if (millis() - lastSend > 200) {
    char buf[128];
    snprintf(buf, sizeof(buf),
      "{\"p1\":%.2f,\"p2\":%.2f,\"dp\":%.4f}", P1, P2, deltaP);
    events.send(buf, "message");

    Serial.print("ΔP: ");
    Serial.print(deltaP, 4);
    Serial.print(" | State: ");
    Serial.print(valveStateToString(valveState));
    Serial.print(" | Triggered: ");
    Serial.println(pressureTriggered);

    lastSend = millis();
  }

  unsigned long now = millis();

  /* ===== EDGE-TRIGGER LOGIC ===== */
  if (abs(deltaP) <= DP_THRESHOLD) {
    pressureTriggered = false;
    valveState = IDLE;
    openCount = 0;
    digitalWrite(VALVE_PIN, VALVE_OFF);
    return;
  }

  if (!pressureTriggered) {
    pressureTriggered = true;
    valveState = IDLE;
  }

  /* ===== FSM ===== */
  switch (valveState) {
    case IDLE:
      valveState = OPEN_PULSE;
      stateStartTime = now;
      break;

    case OPEN_PULSE:
      digitalWrite(VALVE_PIN, VALVE_ON);
      if (now - stateStartTime >= ON_TIME) {
        digitalWrite(VALVE_PIN, VALVE_OFF);
        valveState = CLOSE_PULSE;
        stateStartTime = now;
      }
      break;

    case CLOSE_PULSE:
      if (now - stateStartTime >= OFF_TIME) {
        openCount++;
        valveState = (openCount >= 2) ? WAIT_PHASE : OPEN_PULSE;
        stateStartTime = now;
      }
      break;

    case WAIT_PHASE:
      if (now - stateStartTime >= waitTimeMs) {
        valveState = IDLE;
        openCount = 0;
      }
      break;
  }
}