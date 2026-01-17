#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// ---------- WiFi CONFIG ----------
const char* ssid     = "<<SSID>>";
const char* password = "<<Password>>";

// ---------- POSE DEFINITIONS (EDIT HERE) ----------
struct Pose {
  int base;
  int shoulder;
  int elbow;
  int gripper;
};

const Pose POSE_HOME  = { 65, 155, 180, 100 };  // default
const Pose POSE_GRAB  = { 30,   5,  68,  8 };
const Pose POSE_PICK  = { 30,  65,  68,  8 };
const Pose POSE_PLACE = {110,  5,  72, 100 };

// Pin definitions
const int PIN_BASE     = 13;
const int PIN_SHOULDER = 12;
const int PIN_ELBOW    = 14;
const int PIN_GRIPPER  = 27;

// HOME pose constants (from POSE_HOME)
const int HOME_BASE     = POSE_HOME.base;
const int HOME_SHOULDER = POSE_HOME.shoulder;
const int HOME_ELBOW    = POSE_HOME.elbow;
const int HOME_GRIPPER  = POSE_HOME.gripper;

// Servo objects
Servo servoBase, servoShoulder, servoElbow, servoGripper;

// Smooth motion variables (DEFAULT POSE = HOME)
int currentBase     = HOME_BASE,     targetBase     = HOME_BASE;
int currentShoulder = HOME_SHOULDER, targetShoulder = HOME_SHOULDER;
int currentElbow    = HOME_ELBOW,    targetElbow    = HOME_ELBOW;
int currentGripper  = HOME_GRIPPER,  targetGripper  = HOME_GRIPPER;

WebServer server(80);

// ---------- FORWARD DECLARATIONS ----------
void handleRoot();
void handleSet();
void handlePreset();
void handleSerial();
void smoothMoveAll();
void waitComplete();
void homeAll();
void moveSequential(const Pose &p);
void moveSequentialGrab(const Pose &p);
void moveHomeSequential();
int  lerp(int start, int end, float pct);

// ---------- SETUP / LOOP ----------
void setup() {
  Serial.begin(115200);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected! IP: http://");
  Serial.println(WiFi.localIP());

  servoBase.attach(PIN_BASE, 500, 2400);
  servoShoulder.attach(PIN_SHOULDER, 500, 2400);
  servoElbow.attach(PIN_ELBOW, 500, 2400);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);

  homeAll();  // go to HOME on boot

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/preset", handlePreset);
  server.begin();
  Serial.println("Web control ready!");
}

void loop() {
  server.handleClient();
  handleSerial();
  smoothMoveAll();
}

// ---------- HTTP HANDLERS ----------
void handleRoot() {
  // HTML text uses pose values so UI matches globals
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>ESP32 Robotic Arm</title>
<meta name="viewport" content="width=device-width,initial-scale=1">
<style>
body {
  font-family: Arial, sans-serif;
  text-align: center;
  padding: 20px;
  background: #f5f7fb;
}
h2 {
  color: #333;
  margin-bottom: 10px;
}
.subtitle {
  color: #666;
  font-size: 13px;
  margin-bottom: 18px;
}
.slider-container { margin: 16px 0; }
label {
  display: block;
  margin: 6px 0 4px 0;
  font-weight: bold;
  font-size: 15px;
}
.slider {
  width: 300px;
  height: 8px;
  border-radius: 4px;
  -webkit-appearance: none;
  background: #ddd;
  outline: none;
}
.slider::-webkit-slider-thumb {
  -webkit-appearance: none;
  appearance: none;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  background: #00BCD4;
  border: 2px solid #0288D1;
  cursor: pointer;
}
.value {
  font-size: 16px;
  font-weight: bold;
  color: #00BCD4;
}
button {
  background: #FFB300;
  color: #fff;
  padding: 10px 20px;
  border: none;
  border-radius: 6px;
  margin: 8px;
  cursor: pointer;
  font-size: 15px;
  box-shadow: 0 2px 0 #CC8A00;
}
button:disabled {
  background: #b0bec5;
  box-shadow: none;
  cursor: not-allowed;
}
button:hover:not(:disabled) {
  background: #FFC107;
}
.preset-info {
  background: #fff3cd;
  border: 1px solid #ffe58f;
  color: #8a6d3b;
  padding: 8px 12px;
  border-radius: 6px;
  margin: 16px 0 20px 0;
  font-size: 13px;
}
.hint {
  font-size: 11px;
  color: #666;
  margin-top: 3px;
}
</style>
</head>
<body>
<h2>ESP32 Robotic Arm Control</h2>
<div class="subtitle">Flat yellow theme, live servo control</div>

<div class="preset-info">
HOME:  B65 S155 E180 G100 |
GRAB:  B30 S5  E72  G10  |
PICK:  B30 S65 E72  G10  |
PLACE: B110 S26 E84  G100
</div>

<div class="slider-container">
  <label>Base (Pin 13): <span class="value" id="baseVal">65</span> &deg;</label>
  <input type="range" min="0" max="180" value="65" class="slider" id="baseSlider"
         onchange="update('base',this.value)">
  <div class="hint">0&deg; = full left, 180&deg; = full right</div>
</div>

<div class="slider-container">
  <label>Shoulder (Pin 12): <span class="value" id="shoulderVal">155</span> &deg;</label>
  <input type="range" min="0" max="180" value="155" class="slider" id="shoulderSlider"
         onchange="update('shoulder',this.value)">
  <div class="hint">0&deg; = down, 180&deg; = up</div>
</div>

<div class="slider-container">
  <label>Elbow (Pin 14): <span class="value" id="elbowVal">180</span> &deg;</label>
  <input type="range" min="0" max="180" value="180" class="slider" id="elbowSlider"
         onchange="update('elbow',this.value)">
  <div class="hint">0&deg; = straight, 180&deg; = fully bent</div>
</div>

<div class="slider-container">
  <label>Gripper (Pin 27): <span class="value" id="gripperVal">100</span> &deg;</label>
  <input type="range" min="0" max="180" value="100" class="slider" id="gripperSlider"
         onchange="update('gripper',this.value)">
  <div class="hint">0&deg; = open, 180&deg; = closed</div>
</div>

<br>
<button id="btnHome"  onclick="home()">HOME</button>
<button id="btnGrab"  onclick="grab()">GRAB</button>
<button id="btnPick"  onclick="pick()">PICK</button>
<button id="btnPlace" onclick="place()">PLACE</button>

<script>
function setSliders(b, s, e, g) {
  document.getElementById('baseSlider').value = b;
  document.getElementById('shoulderSlider').value = s;
  document.getElementById('elbowSlider').value = e;
  document.getElementById('gripperSlider').value = g;

  document.getElementById('baseVal').innerHTML = b;
  document.getElementById('shoulderVal').innerHTML = s;
  document.getElementById('elbowVal').innerHTML = e;
  document.getElementById('gripperVal').innerHTML = g;
}

function setButtonsEnabled(enabled) {
  document.getElementById('btnHome').disabled  = !enabled;
  document.getElementById('btnGrab').disabled  = !enabled;
  document.getElementById('btnPick').disabled  = !enabled;
  document.getElementById('btnPlace').disabled = !enabled;
}

function update(servo, value) {
  document.getElementById(servo + 'Val').innerHTML = value;
  fetch('/set?' + servo + '=' + value);
}

// Generic helper: send preset, update sliders, disable buttons temporarily
function runPreset(name, b, s, e, g, durationMs) {
  if (document.getElementById('btnHome').disabled) return;
  setButtonsEnabled(false);
  fetch('/preset?pos=' + name);
  setSliders(b, s, e, g);
  setTimeout(() => setButtonsEnabled(true), durationMs);
}

function home() {
  runPreset('home', 65, 155, 180, 100, 3500);
}

function grab() {
  runPreset('grab', 30, 5, 72, 10, 3500);
}

function pick() {
  runPreset('pick', 30, 65, 72, 10, 3500);
}

function place() {
  runPreset('place', 110, 26, 84, 100, 3500);
}

// ensure sliders match HOME (boot pose) on initial load
document.addEventListener('DOMContentLoaded', function () {
  setSliders(65, 155, 180, 100);
});
</script>
</body>
</html>
)rawliteral";

  server.sendHeader("Content-Type", "text/html; charset=utf-8");
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("base"))     targetBase     = constrain(server.arg("base").toInt(), 0, 180);
  if (server.hasArg("shoulder")) targetShoulder = constrain(server.arg("shoulder").toInt(), 0, 180);
  if (server.hasArg("elbow"))    targetElbow    = constrain(server.arg("elbow").toInt(), 0, 180);
  if (server.hasArg("gripper"))  targetGripper  = constrain(server.arg("gripper").toInt(), 0, 180);
  server.send(200, "text/plain", "OK");
}

// ---------- PRESETS & MOTION ----------
void handlePreset() {
  String pos = server.arg("pos");
  if (pos == "home") {
    moveHomeSequential();
  } 
  else if (pos == "grab") {
    moveSequentialGrab(POSE_GRAB);
  }
  else if (pos == "pick") {
    moveSequential(POSE_PICK);
  }
  else if (pos == "place") {
    moveSequential(POSE_PLACE);
  }
  server.send(200, "text/plain", "OK");
}

// HOME: Shoulder -> Elbow -> Gripper -> Base (base last)
void moveHomeSequential() {
  targetShoulder = HOME_SHOULDER; waitComplete();
  targetElbow    = HOME_ELBOW;    waitComplete();
  targetGripper  = HOME_GRIPPER;  waitComplete();
  targetBase     = HOME_BASE;     waitComplete();
}

// Generic sequential: Base → Shoulder → Elbow → Gripper
void moveSequential(const Pose &p) {
  targetBase     = p.base;     waitComplete();
  targetShoulder = p.shoulder; waitComplete();
  targetElbow    = p.elbow;    waitComplete();
  targetGripper  = p.gripper;  waitComplete();
}

// GRAB: Base → Elbow → Shoulder → Gripper
void moveSequentialGrab(const Pose &p) {
  targetBase     = p.base;     waitComplete();
  targetElbow    = p.elbow;    waitComplete();
  targetShoulder = p.shoulder; waitComplete();
  targetGripper  = p.gripper;  waitComplete();
}

void waitComplete() {
  unsigned long startTime = millis();
  while (millis() - startTime < 4000) {
    smoothMoveAll();
    if (abs(currentBase     - targetBase)     < 2 &&
        abs(currentShoulder - targetShoulder) < 2 &&
        abs(currentElbow    - targetElbow)    < 2 &&
        abs(currentGripper  - targetGripper)  < 2) {
      delay(150);
      return;
    }
  }
}

void handleSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  cmd.toLowerCase();

  if (cmd.startsWith("b"))      targetBase     = constrain(cmd.substring(1).toInt(), 0, 180);
  else if (cmd.startsWith("s")) targetShoulder = constrain(cmd.substring(1).toInt(), 0, 180);
  else if (cmd.startsWith("e")) targetElbow    = constrain(cmd.substring(1).toInt(), 0, 180);
  else if (cmd.startsWith("g")) targetGripper  = constrain(cmd.substring(1).toInt(), 0, 180);
  else if (cmd == "home")  moveHomeSequential();
  else if (cmd == "grab")  moveSequentialGrab(POSE_GRAB);
  else if (cmd == "pick")  moveSequential(POSE_PICK);
  else if (cmd == "place") moveSequential(POSE_PLACE);

  Serial.println("Targets: B"+String(targetBase)+" S"+String(targetShoulder)+
                 " E"+String(targetElbow)+" G"+String(targetGripper));
}

// ---------- SMOOTH MOTION ----------
void smoothMoveAll() {
  // Base slower to reduce jerk; others a bit faster
  currentBase     = lerp(currentBase,     targetBase,     0.05);
  currentShoulder = lerp(currentShoulder, targetShoulder, 0.12);
  currentElbow    = lerp(currentElbow,    targetElbow,    0.12);
  currentGripper  = lerp(currentGripper,  targetGripper,  0.12);

  servoBase.write(currentBase);
  servoShoulder.write(currentShoulder);
  servoElbow.write(currentElbow);
  servoGripper.write(currentGripper);

  delay(15);
}

void homeAll() {
  moveHomeSequential();
}

int lerp(int start, int end, float pct) {
  return start + (end - start) * pct;
}
