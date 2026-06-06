/*
  ESP32 robot firmware + SR04 front obstacle sensor
  Pins from your photo:
    FRONT_TRIG = GPIO13
    FRONT_ECHO = GPIO12

  Supported commands from the Ubuntu software, by Serial or UDP port 8080:
    PING
    STOP
    <left_pwm>,<right_pwm>        example: 105,-105
    SR04?                         read distance once
    OBS,ON / OBS,OFF
    OBS,CFG,<en>,<threshold_cm>,<speed_pwm>,<turn_ms>,<straight_ms>
    OBS,TH,<cm>
    OBS,SPD,<pwm>
    OBS,TURN_MS,<ms>
    OBS,STRAIGHT_MS,<ms>

  Obstacle logic:
    If OBS is ON and measured distance <= threshold:
    robot turns right for turn_ms, then goes forward for straight_ms,
    then returns to normal line-follow commands from the PC software.
*/

#include <WiFi.h>
#include <WiFiUdp.h>

// ===== WiFi config =====
// Fill in your router WiFi. If SSID is empty, ESP32 starts AP mode.
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";
const char* AP_SSID   = "ESP32_ROBOT";
const char* AP_PASS   = "12345678";
const uint16_t UDP_PORT = 8080;
WiFiUDP udp;
IPAddress lastRemoteIP;
uint16_t lastRemotePort = 0;

// ===== Motor pins =====
const int LEFT_EN  = 14;
const int LEFT_IN1 = 26;
const int LEFT_IN2 = 25;
const int RIGHT_EN  = 27;
const int RIGHT_IN1 = 33;
const int RIGHT_IN2 = 32;

// ===== SR04 front sensor pins =====
const int FRONT_TRIG = 13;
const int FRONT_ECHO = 12;

// ===== Obstacle parameters =====
bool obsEnabled = false;
float obsThresholdCm = 25.0;
int obsSpeedPwm = 110;
unsigned long obsTurnMs = 650;
unsigned long obsStraightMs = 900;

float lastDistanceCm = -1.0;
unsigned long lastMeasureMs = 0;
unsigned long lastReportMs = 0;
unsigned long ignoreUntilMs = 0;

enum AvoidPhase { AVOID_NONE, AVOID_TURN_RIGHT, AVOID_FORWARD };
AvoidPhase avoidPhase = AVOID_NONE;
unsigned long avoidPhaseEndMs = 0;

String serialLine;

int clipPwm(int v) {
  if (v > 255) return 255;
  if (v < -255) return -255;
  return v;
}

void motorOne(int en, int in1, int in2, int pwm) {
  pwm = clipPwm(pwm);
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(en, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(en, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 0);
  }
}

void setMotors(int left, int right) {
  motorOne(LEFT_EN, LEFT_IN1, LEFT_IN2, left);
  motorOne(RIGHT_EN, RIGHT_IN1, RIGHT_IN2, right);
}

void stopMotors() {
  setMotors(0, 0);
}

void sendReply(const String& msg) {
  Serial.println(msg);
  if (lastRemotePort != 0) {
    udp.beginPacket(lastRemoteIP, lastRemotePort);
    udp.print(msg);
    udp.endPacket();
  }
}

float readFrontCm() {
  digitalWrite(FRONT_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(FRONT_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(FRONT_TRIG, LOW);

  unsigned long duration = pulseIn(FRONT_ECHO, HIGH, 30000UL); // 30 ms ~ 5 m
  if (duration == 0) return -1.0;
  return duration / 58.0;
}

void startAvoid() {
  avoidPhase = AVOID_TURN_RIGHT;
  avoidPhaseEndMs = millis() + obsTurnMs;
  setMotors(obsSpeedPwm, -obsSpeedPwm); // same as software XOAY PHAI TEST
  sendReply("OBS,AVOID,START");
}

void updateAvoid() {
  if (avoidPhase == AVOID_NONE) return;

  unsigned long now = millis();
  if (avoidPhase == AVOID_TURN_RIGHT) {
    if (now < avoidPhaseEndMs) {
      setMotors(obsSpeedPwm, -obsSpeedPwm);
      return;
    }
    avoidPhase = AVOID_FORWARD;
    avoidPhaseEndMs = now + obsStraightMs;
    setMotors(obsSpeedPwm, obsSpeedPwm);
    sendReply("OBS,AVOID,FORWARD");
    return;
  }

  if (avoidPhase == AVOID_FORWARD) {
    if (now < avoidPhaseEndMs) {
      setMotors(obsSpeedPwm, obsSpeedPwm);
      return;
    }
    avoidPhase = AVOID_NONE;
    ignoreUntilMs = now + 900;
    stopMotors();
    sendReply("OBS,AVOID,DONE");
  }
}

void parseObsCommand(String cmd) {
  cmd.trim();
  String up = cmd;
  up.toUpperCase();

  if (up == "OBS,ON") {
    obsEnabled = true;
    sendReply("OBS,ON,OK");
    return;
  }
  if (up == "OBS,OFF") {
    obsEnabled = false;
    avoidPhase = AVOID_NONE;
    stopMotors();
    sendReply("OBS,OFF,OK");
    return;
  }
  if (up == "OBS,READ" || up == "SR04?") {
    lastDistanceCm = readFrontCm();
    sendReply("SR04,DIST," + String(lastDistanceCm, 1));
    return;
  }

  // Split CSV
  const int MAX_PARTS = 8;
  String parts[MAX_PARTS];
  int count = 0;
  int start = 0;
  while (count < MAX_PARTS) {
    int idx = cmd.indexOf(',', start);
    if (idx < 0) {
      parts[count++] = cmd.substring(start);
      break;
    }
    parts[count++] = cmd.substring(start, idx);
    start = idx + 1;
  }
  for (int i = 0; i < count; i++) parts[i].trim();

  if (count >= 6 && parts[0].equalsIgnoreCase("OBS") && parts[1].equalsIgnoreCase("CFG")) {
    obsEnabled = parts[2].toInt() != 0;
    obsThresholdCm = parts[3].toFloat();
    obsSpeedPwm = constrain(parts[4].toInt(), 40, 255);
    obsTurnMs = constrain(parts[5].toInt(), 100, 6000);
    if (count >= 7) obsStraightMs = constrain(parts[6].toInt(), 100, 8000);
    sendReply("OBS,CFG,OK");
    return;
  }
  if (count >= 3 && parts[0].equalsIgnoreCase("OBS")) {
    if (parts[1].equalsIgnoreCase("TH")) obsThresholdCm = parts[2].toFloat();
    else if (parts[1].equalsIgnoreCase("SPD")) obsSpeedPwm = constrain(parts[2].toInt(), 40, 255);
    else if (parts[1].equalsIgnoreCase("TURN_MS")) obsTurnMs = constrain(parts[2].toInt(), 100, 6000);
    else if (parts[1].equalsIgnoreCase("STRAIGHT_MS")) obsStraightMs = constrain(parts[2].toInt(), 100, 8000);
    sendReply("OBS,PARAM,OK");
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  String up = cmd;
  up.toUpperCase();

  if (up == "PING") { sendReply("PONG"); return; }
  if (up == "STOP") { avoidPhase = AVOID_NONE; stopMotors(); sendReply("STOP,OK"); return; }
  if (up.startsWith("OBS") || up == "SR04?") { parseObsCommand(cmd); return; }

  // During obstacle avoidance, ignore PC PWM so the turn/forward sequence is not overwritten.
  if (avoidPhase != AVOID_NONE) {
    sendReply("OBS,AVOID,ACTIVE");
    return;
  }

  int comma = cmd.indexOf(',');
  if (comma > 0) {
    int left = clipPwm(cmd.substring(0, comma).toInt());
    int right = clipPwm(cmd.substring(comma + 1).toInt());
    setMotors(left, right);
    return;
  }

  sendReply("ERR,UNKNOWN_CMD," + cmd);
}

void setupWiFi() {
  if (String(WIFI_SSID).length() > 0) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) delay(250);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("WiFi STA IP: "); Serial.println(WiFi.localIP());
      return;
    }
  }
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("WiFi AP IP: "); Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT); pinMode(LEFT_IN2, OUTPUT); pinMode(LEFT_EN, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT); pinMode(RIGHT_IN2, OUTPUT); pinMode(RIGHT_EN, OUTPUT);
  stopMotors();

  pinMode(FRONT_TRIG, OUTPUT);
  pinMode(FRONT_ECHO, INPUT);
  digitalWrite(FRONT_TRIG, LOW);

  setupWiFi();
  udp.begin(UDP_PORT);
  Serial.println("ESP32 ROBOT SR04 READY UDP 8080");
}

void loop() {
  // Serial commands
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      handleCommand(serialLine);
      serialLine = "";
    } else {
      serialLine += c;
      if (serialLine.length() > 120) serialLine = "";
    }
  }

  // UDP commands
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    char buf[160];
    int n = udp.read(buf, sizeof(buf) - 1);
    if (n > 0) {
      buf[n] = 0;
      lastRemoteIP = udp.remoteIP();
      lastRemotePort = udp.remotePort();
      handleCommand(String(buf));
    }
  }

  unsigned long now = millis();

  // Measure and report SR04 periodically
  if (now - lastMeasureMs >= 80) {
    lastMeasureMs = now;
    lastDistanceCm = readFrontCm();
  }
  if (now - lastReportMs >= 200) {
    lastReportMs = now;
    sendReply("SR04,DIST," + String(lastDistanceCm, 1));
  }

  updateAvoid();

  if (obsEnabled && avoidPhase == AVOID_NONE && now > ignoreUntilMs && lastDistanceCm > 0 && lastDistanceCm <= obsThresholdCm) {
    startAvoid();
  }
}
