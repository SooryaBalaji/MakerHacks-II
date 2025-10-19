#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// === Pins ===
#define TRIG_PIN 18
#define ECHO_PIN 5
#define SERVO_PIN 27

// === Wi-Fi Credentials ===
const char* ssid = "ESP32-siva_the_goat";
const char* password = "robot1234";

// === PID Parameters ===
float D0 = 15.0;   // Target distance (cm)
float kP = 0.5;
float kI = 0.0;
float kD = 0.0;

// === Globals ===
long duration;
float distance = 0;
float previousError = 0;
float integral = 0;
float servoPosition = 90;
float minPositionForServo = 0;
float maxPositionForServo = 180;

// PID Control Rate
unsigned long lastPIDUpdate = 0;
const unsigned long PID_INTERVAL = 20; // 50Hz update rate

Servo flywheelServo;
WebServer server(80);

// === Measure distance ===
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN, HIGH, 40000);
  if (duration == 0) {
    Serial.println("⚠️ No echo detected");
    return distance; // Return last valid distance
  }

  float dist = (duration * 0.0343) / 2.0;
  
  // Filter out unreasonable readings
  if (dist > 0 && dist < 100) {
    return dist;
  }
  return distance;
}

// === PID Control Update ===
void updatePID() {
  distance = getDistance();

  // PID calculation
  float tiltError = distance - D0;
  
  // Deadband to prevent jitter
  if (fabs(tiltError) < 0.5) {
    tiltError = 0;
  }

  // Integral with anti-windup
  integral += tiltError;
  integral = constrain(integral, -100, 100);

  float derivative = tiltError - previousError;
  previousError = tiltError;
  
  float outputValue = (kP * tiltError) + (kI * integral) + (kD * derivative);

  // === Servo Control ===
  servoPosition = constrain(servoPosition - outputValue, minPositionForServo, maxPositionForServo);
  flywheelServo.write(servoPosition);
}

// === Handle JSON endpoint ===
void handleData() {
  StaticJsonDocument<300> doc;
  doc["distance"] = distance;
  doc["servo"] = servoPosition;
  doc["tiltError"] = distance - D0;
  doc["timestamp"] = millis();
  doc["kP"] = kP;
  doc["kI"] = kI;
  doc["kD"] = kD;
  doc["target"] = D0;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

// === Update PID Parameters ===
void handleUpdatePID() {
  if (server.hasArg("plain")) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if (!error) {
      if (doc.containsKey("kP")) kP = doc["kP"];
      if (doc.containsKey("kI")) kI = doc["kI"];
      if (doc.containsKey("kD")) kD = doc["kD"];
      if (doc.containsKey("target")) D0 = doc["target"];
      
      server.send(200, "application/json", "{\"status\":\"ok\"}");
      Serial.printf("PID Updated: kP=%.2f, kI=%.2f, kD=%.2f, Target=%.2f\n", kP, kI, kD, D0);
      return;
    }
  }
  server.send(400, "application/json", "{\"status\":\"error\"}");
}

// === Reset System ===
void handleReset() {
  servoPosition = 90;
  integral = 0;
  previousError = 0;
  flywheelServo.write(servoPosition);
  server.send(200, "application/json", "{\"status\":\"reset\"}");
  Serial.println("System reset to center position");
}

// === Web Page ===
void handleRoot() {
  String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Flywheel Balance Robot</title>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body {
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif;
      background: #ffffff;
      min-height: 100vh;
      padding: 40px 20px;
      color: #1a1a1a;
    }
    .container {
      max-width: 800px;
      margin: 0 auto;
    }
    h1 {
      color: #1a1a1a;
      text-align: center;
      margin-bottom: 40px;
      font-size: 32px;
      font-weight: 600;
      letter-spacing: -0.5px;
    }
    .card {
      background: #ffffff;
      border: 1px solid #e5e5e5;
      border-radius: 8px;
      padding: 32px;
      margin-bottom: 24px;
    }
    .card h2 {
      margin-bottom: 24px;
      color: #1a1a1a;
      font-size: 18px;
      font-weight: 600;
      letter-spacing: -0.3px;
    }
    .data-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(160px, 1fr));
      gap: 16px;
      margin-bottom: 24px;
    }
    .data-item {
      background: #fafafa;
      padding: 20px;
      border-radius: 6px;
      border: 1px solid #f0f0f0;
    }
    .data-label {
      font-size: 13px;
      color: #666;
      text-transform: uppercase;
      margin-bottom: 8px;
      font-weight: 500;
      letter-spacing: 0.5px;
    }
    .data-value {
      font-size: 28px;
      font-weight: 600;
      color: #1a1a1a;
      line-height: 1;
    }
    .data-unit {
      font-size: 13px;
      color: #999;
      margin-top: 4px;
    }
    .control-group {
      margin-bottom: 20px;
    }
    .control-group label {
      display: block;
      margin-bottom: 8px;
      font-weight: 500;
      color: #333;
      font-size: 14px;
    }
    .control-group input {
      width: 100%;
      padding: 12px 16px;
      border: 1px solid #e5e5e5;
      border-radius: 6px;
      font-size: 15px;
      transition: all 0.2s;
      background: #fafafa;
    }
    .control-group input:focus {
      outline: none;
      border-color: #1a1a1a;
      background: #ffffff;
    }
    .button-group {
      display: flex;
      gap: 12px;
      margin-top: 24px;
    }
    button {
      flex: 1;
      padding: 14px 24px;
      border: none;
      border-radius: 6px;
      font-size: 15px;
      font-weight: 500;
      cursor: pointer;
      transition: all 0.2s;
    }
    button:hover {
      opacity: 0.9;
    }
    button:active {
      transform: scale(0.98);
    }
    .btn-primary {
      background: #1a1a1a;
      color: white;
    }
    .btn-secondary {
      background: #f5f5f5;
      color: #1a1a1a;
      border: 1px solid #e5e5e5;
    }
    .status {
      text-align: center;
      padding: 12px;
      border-radius: 6px;
      margin-top: 16px;
      font-weight: 500;
      font-size: 14px;
      display: none;
    }
    .status.success {
      background: #f0f9f4;
      color: #166534;
      border: 1px solid #bbf7d0;
      display: block;
    }
    .status.error {
      background: #fef2f2;
      color: #991b1b;
      border: 1px solid #fecaca;
      display: block;
    }
    @media (max-width: 600px) {
      body { padding: 20px 16px; }
      h1 { font-size: 24px; margin-bottom: 24px; }
      .card { padding: 24px; }
      .data-value { font-size: 24px; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>🎯 Flywheel Balance Robot</h1>
    
    <div class="card">
      <h2>Real-time Data</h2>
      <div class="data-grid">
        <div class="data-item">
          <div class="data-label">Distance</div>
          <div class="data-value" id="distance">--</div>
          <div class="data-unit">cm</div>
        </div>
        <div class="data-item">
          <div class="data-label">Flywheel</div>
          <div class="data-value" id="servo">--</div>
          <div class="data-unit">degrees</div>
        </div>
        <div class="data-item">
          <div class="data-label">Error</div>
          <div class="data-value" id="error">--</div>
          <div class="data-unit">cm</div>
        </div>
      </div>
    </div>

    <div class="card">
      <h2>PID Tuning</h2>
      <div class="control-group">
        <label for="targetInput">Target Distance (cm)</label>
        <input type="number" id="targetInput" step="0.5" value="15.0">
      </div>
      <div class="control-group">
        <label for="kpInput">kP (Proportional)</label>
        <input type="number" id="kpInput" step="0.1" value="0.5">
      </div>
      <div class="control-group">
        <label for="kiInput">kI (Integral)</label>
        <input type="number" id="kiInput" step="0.01" value="0.0">
      </div>
      <div class="control-group">
        <label for="kdInput">kD (Derivative)</label>
        <input type="number" id="kdInput" step="0.1" value="0.0">
      </div>
      <div class="button-group">
        <button class="btn-primary" onclick="updatePID()">Update PID</button>
        <button class="btn-secondary" onclick="resetSystem()">Reset</button>
      </div>
      <div class="status" id="status"></div>
    </div>
  </div>

  <script>
    async function updateData() {
      try {
        const res = await fetch('/data');
        const data = await res.json();
        document.getElementById('distance').textContent = data.distance.toFixed(2);
        document.getElementById('servo').textContent = data.servo.toFixed(1);
        document.getElementById('error').textContent = data.tiltError.toFixed(2);
        
        // Update input fields with current values
        document.getElementById('targetInput').value = data.target;
        document.getElementById('kpInput').value = data.kP;
        document.getElementById('kiInput').value = data.kI;
        document.getElementById('kdInput').value = data.kD;
      } catch (e) {
        console.log('Fetch error:', e);
      }
    }

    async function updatePID() {
      const params = {
        target: parseFloat(document.getElementById('targetInput').value),
        kP: parseFloat(document.getElementById('kpInput').value),
        kI: parseFloat(document.getElementById('kiInput').value),
        kD: parseFloat(document.getElementById('kdInput').value)
      };

      try {
        const res = await fetch('/update_pid', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(params)
        });
        const data = await res.json();
        showStatus(data.status === 'ok' ? 'PID parameters updated!' : 'Update failed!', data.status === 'ok');
      } catch (e) {
        showStatus('Update failed!', false);
      }
    }

    async function resetSystem() {
      try {
        const res = await fetch('/reset');
        const data = await res.json();
        showStatus('System reset!', true);
      } catch (e) {
        showStatus('Reset failed!', false);
      }
    }

    function showStatus(message, success) {
      const status = document.getElementById('status');
      status.textContent = message;
      status.className = 'status ' + (success ? 'success' : 'error');
      setTimeout(() => {
        status.style.display = 'none';
      }, 3000);
    }

    setInterval(updateData, 200);
    updateData();
  </script>
</body>
</html>
  )rawliteral";

  server.send(200, "text/html", page);
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  flywheelServo.attach(SERVO_PIN);
  flywheelServo.write(servoPosition);

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("\n================================");
  Serial.println("Flywheel Balance Robot Started!");
  Serial.println("================================");
  Serial.print("SSID: "); Serial.println(ssid);
  Serial.print("Password: "); Serial.println(password);
  Serial.print("AP IP Address: "); Serial.println(WiFi.softAPIP());
  Serial.println("================================");

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/update_pid", HTTP_POST, handleUpdatePID);
  server.on("/reset", handleReset);
  
  server.begin();
  Serial.println("🚀 Web server running!");
  Serial.println("Connect to WiFi and visit: http://" + WiFi.softAPIP().toString());
}

// === Loop ===
void loop() {
  // Run PID control at consistent rate (50Hz)
  unsigned long currentMillis = millis();
  if (currentMillis - lastPIDUpdate >= PID_INTERVAL) {
    lastPIDUpdate = currentMillis;
    updatePID();
  }
  
  // Handle web requests
  server.handleClient();
}
