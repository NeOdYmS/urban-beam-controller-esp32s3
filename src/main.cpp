#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <AccelStepper.h>

namespace {

struct MotorPins {
  uint8_t step;
  uint8_t dir;
  uint8_t enable;
  uint8_t limitMin;
  uint8_t limitMax;
};

enum AxisId : uint8_t {
  AXIS_DIPOLE = 0,
  AXIS_REFLECTOR = 1,
};

// Update these pin mappings to your final PCB routing.
MotorPins motorPins[2] = {
    {4, 5, 6, 7, 15},
    {16, 17, 18, 8, 3},
};

constexpr uint8_t FORWARD_POWER_ADC_PIN = 1;
constexpr uint8_t REFLECTED_POWER_ADC_PIN = 2;

// Ethernet (W5500 over SPI) pins for ESP32-S3.
constexpr uint8_t ETH_MISO_PIN = 13;
constexpr uint8_t ETH_MOSI_PIN = 11;
constexpr uint8_t ETH_SCK_PIN = 12;
constexpr uint8_t ETH_CS_PIN = 10;
constexpr int8_t ETH_RST_PIN = 9;

constexpr float MAX_SPEED_STEPS_S = 1800.0f;
constexpr float ACCEL_STEPS_S2 = 1200.0f;
constexpr long MAX_TRAVEL_STEPS[2] = {120000, 120000};

// These coefficients map theoretical length (mm) to mechanics (steps).
// Tune on real hardware after first movements.
constexpr float DIPOLE_REF_LENGTH_MM = 9960.0f;
constexpr float REFLECTOR_REF_LENGTH_MM = 9600.0f;
constexpr long DIPOLE_REF_STEPS = 42000;
constexpr long REFLECTOR_REF_STEPS = 45000;
constexpr float DIPOLE_STEPS_PER_MM = -3.8f;
constexpr float REFLECTOR_STEPS_PER_MM = -3.5f;

constexpr float SWR_TARGET = 1.30f;
constexpr float SWR_HIGH_MARGIN = 0.10f;
constexpr float SWR_IMPROVEMENT_HYST = 0.02f;
constexpr long AUTO_MIN_STEP = 8;
constexpr long AUTO_MAX_STEP = 240;

constexpr uint32_t STATE_SAMPLE_MS = 300;
constexpr uint32_t AUTO_UPDATE_MS = 900;

constexpr float ADC_VREF = 3.30f;
constexpr int ADC_MAX = 4095;

byte ETH_MAC[] = {0x02, 0x55, 0x42, 0x10, 0x20, 0x33};
IPAddress fallbackIp(192, 168, 1, 70);
IPAddress fallbackDns(192, 168, 1, 1);
IPAddress fallbackGw(192, 168, 1, 1);
IPAddress fallbackMask(255, 255, 255, 0);

EthernetServer ethServer(80);

class Axis {
public:
  explicit Axis(const MotorPins &pins)
      : pins_(pins), stepper_(AccelStepper::DRIVER, pins.step, pins.dir) {}

  void begin() {
    pinMode(pins_.enable, OUTPUT);
    digitalWrite(pins_.enable, LOW);

    pinMode(pins_.limitMin, INPUT_PULLUP);
    pinMode(pins_.limitMax, INPUT_PULLUP);

    stepper_.setMaxSpeed(MAX_SPEED_STEPS_S);
    stepper_.setAcceleration(ACCEL_STEPS_S2);
    stepper_.setCurrentPosition(0);
    homing_ = false;
    target_ = 0;
  }

  void update(long maxTravel) {
    if (homing_) {
      if (limitMinTriggered()) {
        homing_ = false;
        stepper_.stop();
        stepper_.setCurrentPosition(0);
        target_ = 0;
      } else {
        stepper_.runSpeed();
      }
      return;
    }

    long current = stepper_.currentPosition();
    if (current <= 0 && stepper_.distanceToGo() < 0) {
      stepper_.setCurrentPosition(0);
      stepper_.moveTo(0);
      target_ = 0;
      return;
    }
    if (current >= maxTravel && stepper_.distanceToGo() > 0) {
      stepper_.setCurrentPosition(maxTravel);
      stepper_.moveTo(maxTravel);
      target_ = maxTravel;
      return;
    }

    if (stepper_.distanceToGo() < 0 && limitMinTriggered()) {
      stepper_.setCurrentPosition(0);
      stepper_.moveTo(0);
      target_ = 0;
      return;
    }
    if (stepper_.distanceToGo() > 0 && limitMaxTriggered()) {
      long lock = stepper_.currentPosition();
      stepper_.setCurrentPosition(lock);
      stepper_.moveTo(lock);
      target_ = lock;
      return;
    }

    stepper_.run();
  }

  void home() {
    homing_ = true;
    stepper_.setSpeed(-700.0f);
  }

  void moveTo(long absoluteSteps, long maxTravel) {
    long clamped = constrain(absoluteSteps, 0L, maxTravel);
    if (limitMinTriggered() && clamped < stepper_.currentPosition()) {
      clamped = stepper_.currentPosition();
    }
    if (limitMaxTriggered() && clamped > stepper_.currentPosition()) {
      clamped = stepper_.currentPosition();
    }
    target_ = clamped;
    stepper_.moveTo(clamped);
  }

  void jog(long delta, long maxTravel) {
    moveTo(stepper_.currentPosition() + delta, maxTravel);
  }

  void stop() {
    homing_ = false;
    stepper_.stop();
    target_ = stepper_.currentPosition();
  }

  long position() { return stepper_.currentPosition(); }
  long target() const { return target_; }
  bool busy() { return homing_ || stepper_.distanceToGo() != 0; }
  bool limitMinTriggered() const { return digitalRead(pins_.limitMin) == LOW; }
  bool limitMaxTriggered() const { return digitalRead(pins_.limitMax) == LOW; }

private:
  MotorPins pins_;
  AccelStepper stepper_;
  bool homing_ = false;
  long target_ = 0;
};

Axis axisDipole(motorPins[AXIS_DIPOLE]);
Axis axisReflector(motorPins[AXIS_REFLECTOR]);

float currentFrequencyMHz = 7.10f;
float currentSwr = 99.0f;
float autoBestSwr = 99.0f;
bool autoTuneEnabled = false;
uint8_t autoAxisRoundRobin = 0;
int8_t autoDirection[2] = {1, 1};
long autoStep[2] = {60, 60};
uint32_t lastStateSampleMs = 0;
uint32_t lastAutoUpdateMs = 0;

float targetLengthDipoleMM = 0.0f;
float targetLengthReflectorMM = 0.0f;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="fr">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>URBAN BEAM Ethernet Controller</title>
  <style>
    :root {
      --bg1: #0f172a;
      --bg2: #1e293b;
      --card: rgba(255,255,255,0.09);
      --line: #22d3ee;
      --ok: #34d399;
      --warn: #f59e0b;
      --txt: #e2e8f0;
      --muted: #94a3b8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--txt);
      background: radial-gradient(circle at 10% 10%, #1d4ed8 0%, transparent 35%),
                  radial-gradient(circle at 90% 20%, #0ea5e9 0%, transparent 30%),
                  linear-gradient(165deg, var(--bg1), var(--bg2));
      min-height: 100vh;
      padding: 18px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 14px;
    }
    .card {
      background: var(--card);
      border: 1px solid rgba(255,255,255,0.13);
      border-radius: 14px;
      padding: 14px;
      backdrop-filter: blur(8px);
    }
    h1 { margin: 0 0 12px; font-size: 1.45rem; letter-spacing: .02em; }
    h2 { margin: 0 0 8px; font-size: 1rem; color: #f8fafc; }
    .row { display: flex; gap: 8px; flex-wrap: wrap; align-items: center; margin-top: 8px; }
    .badge { font-weight: 700; color: #020617; background: #67e8f9; border-radius: 999px; padding: 4px 10px; }
    .muted { color: var(--muted); }
    button {
      background: rgba(15, 23, 42, .75);
      color: var(--txt);
      border: 1px solid rgba(148,163,184,.5);
      border-radius: 10px;
      padding: 8px 11px;
      cursor: pointer;
    }
    button:hover { border-color: var(--line); }
    input {
      width: 100%;
      max-width: 180px;
      padding: 7px 10px;
      border-radius: 10px;
      border: 1px solid rgba(148,163,184,.5);
      background: rgba(2,6,23,.7);
      color: var(--txt);
    }
    #chart {
      width: 100%;
      height: 240px;
      border-radius: 12px;
      background: rgba(2, 6, 23, 0.65);
      border: 1px solid rgba(148,163,184,.4);
    }
    .kpi { font-size: 1.25rem; font-weight: 700; }
    .ok { color: var(--ok); }
    .warn { color: var(--warn); }
  </style>
</head>
<body>
  <h1>URBAN BEAM - Ethernet dipole + reflector</h1>
  <div class="grid">
    <section class="card">
      <h2>Mesure SWR</h2>
      <div id="swrLabel" class="kpi">SWR: --</div>
      <div class="muted">Frequence: <span id="freq">--</span> MHz</div>
      <div class="muted">Ethernet: <span id="ip">--</span></div>
      <canvas id="chart" width="640" height="260"></canvas>
    </section>

    <section class="card">
      <h2>Mode auto</h2>
      <div class="row">
        <label>Frequence (MHz)</label>
        <input id="freqInput" type="number" min="1" max="60" step="0.01" value="7.10" />
        <button onclick="setFrequency()">Appliquer</button>
      </div>
      <div class="row">
        <button onclick="setAuto(1)">Activer tatonnement SWR</button>
        <button onclick="setAuto(0)">Stop auto</button>
      </div>
      <div class="row muted">Etat: <span id="autoState">--</span></div>
      <div class="row"><span class="badge">Objectif SWR <= 1.30</span></div>
      <div class="row muted">Longueur dipole calculee: <span id="dipoleMM">--</span> mm</div>
      <div class="row muted">Longueur reflector calculee: <span id="reflMM">--</span> mm</div>
    </section>

    <section class="card">
      <h2>Axe dipole</h2>
      <div>Position: <span id="m1pos">--</span> pas</div>
      <div>Fins de course: min <span id="m1min">--</span> / max <span id="m1max">--</span></div>
      <div class="row">
        <button onclick="jog(0,-200)">-200</button>
        <button onclick="jog(0,200)">+200</button>
        <button onclick="home(0)">Home</button>
        <button onclick="stop(0)">Stop</button>
      </div>
    </section>

    <section class="card">
      <h2>Axe reflector</h2>
      <div>Position: <span id="m2pos">--</span> pas</div>
      <div>Fins de course: min <span id="m2min">--</span> / max <span id="m2max">--</span></div>
      <div class="row">
        <button onclick="jog(1,-200)">-200</button>
        <button onclick="jog(1,200)">+200</button>
        <button onclick="home(1)">Home</button>
        <button onclick="stop(1)">Stop</button>
      </div>
    </section>
  </div>

  <script>
    const chart = document.getElementById('chart');
    const ctx = chart.getContext('2d');
    const history = [];

    function api(path) {
      return fetch(path, { cache: 'no-store' }).then(r => r.json());
    }

    function setFrequency() {
      const f = parseFloat(document.getElementById('freqInput').value || '7.1');
      fetch(`/api/frequency?mhz=${encodeURIComponent(f)}`);
    }

    function setAuto(on) { fetch(`/api/auto?on=${on}`); }
    function jog(m, d) { fetch(`/api/jog?motor=${m}&delta=${d}`); }
    function home(m) { fetch(`/api/home?motor=${m}`); }
    function stop(m) { fetch(`/api/stop?motor=${m}`); }

    function drawChart() {
      const w = chart.width;
      const h = chart.height;
      ctx.clearRect(0, 0, w, h);
      ctx.strokeStyle = 'rgba(148,163,184,0.4)';
      ctx.lineWidth = 1;
      for (let y = 0; y <= 4; y++) {
        const yy = 20 + y * ((h - 40) / 4);
        ctx.beginPath();
        ctx.moveTo(30, yy);
        ctx.lineTo(w - 10, yy);
        ctx.stroke();
      }
      if (history.length < 2) return;
      const maxS = Math.max(2.5, ...history.map(v => v.swr));
      const minS = 1.0;
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 2;
      ctx.beginPath();
      history.forEach((p, i) => {
        const x = 30 + (i / (history.length - 1)) * (w - 40);
        const y = h - 20 - ((p.swr - minS) / (maxS - minS)) * (h - 40);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();
    }

    async function refresh() {
      try {
        const s = await api('/api/state');
        document.getElementById('swrLabel').textContent = `SWR: ${s.swr.toFixed(2)}`;
        document.getElementById('swrLabel').className = 'kpi ' + (s.swr <= 1.3 ? 'ok' : 'warn');
        document.getElementById('freq').textContent = s.frequencyMHz.toFixed(2);
        document.getElementById('autoState').textContent = s.autoTune ? 'ACTIF' : 'MANUEL';
        document.getElementById('ip').textContent = s.ethernetIp;
        document.getElementById('dipoleMM').textContent = s.lengthDipoleMM.toFixed(1);
        document.getElementById('reflMM').textContent = s.lengthReflectorMM.toFixed(1);
        document.getElementById('m1pos').textContent = s.motors[0].position;
        document.getElementById('m2pos').textContent = s.motors[1].position;
        document.getElementById('m1min').textContent = s.motors[0].limitMin ? 'ON' : 'OFF';
        document.getElementById('m1max').textContent = s.motors[0].limitMax ? 'ON' : 'OFF';
        document.getElementById('m2min').textContent = s.motors[1].limitMin ? 'ON' : 'OFF';
        document.getElementById('m2max').textContent = s.motors[1].limitMax ? 'ON' : 'OFF';

        history.push({ swr: s.swr });
        if (history.length > 200) history.shift();
        drawChart();
      } catch (_) {}
    }

    setInterval(refresh, 450);
    refresh();
  </script>
</body>
</html>
)HTML";

float quarterWaveDipoleMeters(float frequencyMHz) {
  return ((300.0f / frequencyMHz) * 0.95f) / 4.0f;
}

float quarterWaveReflectorMeters(float frequencyMHz) {
  return ((300.0f / frequencyMHz) * 0.89f) / 4.0f;
}

long dipoleStepsFromLength(float lengthMM) {
  float offset = lengthMM - DIPOLE_REF_LENGTH_MM;
  long target = static_cast<long>(DIPOLE_REF_STEPS + offset * DIPOLE_STEPS_PER_MM);
  return constrain(target, 0L, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
}

long reflectorStepsFromLength(float lengthMM) {
  float offset = lengthMM - REFLECTOR_REF_LENGTH_MM;
  long target = static_cast<long>(REFLECTOR_REF_STEPS + offset * REFLECTOR_STEPS_PER_MM);
  return constrain(target, 0L, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
}

float readADCPinVolts(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += analogRead(pin);
    delayMicroseconds(150);
  }
  float avg = static_cast<float>(sum) / 8.0f;
  return (avg / static_cast<float>(ADC_MAX)) * ADC_VREF;
}

float computeSWR() {
  float vf = readADCPinVolts(FORWARD_POWER_ADC_PIN);
  float vr = readADCPinVolts(REFLECTED_POWER_ADC_PIN);

  // Initial approximation for AD8307-based detector board.
  float rho = vr / max(vf, 0.01f);
  rho = constrain(rho, 0.0f, 0.98f);
  return (1.0f + rho) / (1.0f - rho);
}

void applyFrequencyModel(float freqMHz) {
  currentFrequencyMHz = constrain(freqMHz, 1.0f, 60.0f);

  targetLengthDipoleMM = quarterWaveDipoleMeters(currentFrequencyMHz) * 1000.0f;
  targetLengthReflectorMM = quarterWaveReflectorMeters(currentFrequencyMHz) * 1000.0f;

  long dipoleSteps = dipoleStepsFromLength(targetLengthDipoleMM);
  long reflectorSteps = reflectorStepsFromLength(targetLengthReflectorMM);
  axisDipole.moveTo(dipoleSteps, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  axisReflector.moveTo(reflectorSteps, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);

  autoBestSwr = 99.0f;
  autoStep[AXIS_DIPOLE] = 60;
  autoStep[AXIS_REFLECTOR] = 60;
  autoDirection[AXIS_DIPOLE] = 1;
  autoDirection[AXIS_REFLECTOR] = 1;
}

bool allAxesIdle() {
  return !axisDipole.busy() && !axisReflector.busy();
}

void updateAutoTuning() {
  if (!autoTuneEnabled || !allAxesIdle()) {
    return;
  }

  uint32_t now = millis();
  if (now - lastAutoUpdateMs < AUTO_UPDATE_MS) {
    return;
  }
  lastAutoUpdateMs = now;

  if (currentSwr <= SWR_TARGET + SWR_HIGH_MARGIN) {
    return;
  }

  uint8_t axis = autoAxisRoundRobin % 2;
  autoAxisRoundRobin++;

  bool improved = (currentSwr + SWR_IMPROVEMENT_HYST) < autoBestSwr;
  if (improved) {
    autoBestSwr = currentSwr;
    autoStep[axis] = min(autoStep[axis] + 10, AUTO_MAX_STEP);
  } else {
    autoDirection[axis] = -autoDirection[axis];
    autoStep[axis] = max(autoStep[axis] / 2, AUTO_MIN_STEP);
  }

  long delta = static_cast<long>(autoDirection[axis] * autoStep[axis]);
  if (axis == AXIS_DIPOLE) {
    axisDipole.jog(delta, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  } else {
    axisReflector.jog(delta, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
  }
}

String boolJSON(bool v) {
  return v ? "true" : "false";
}

String getArg(const String &query, const String &key) {
  int from = 0;
  while (from < static_cast<int>(query.length())) {
    int amp = query.indexOf('&', from);
    if (amp < 0) {
      amp = query.length();
    }
    int eq = query.indexOf('=', from);
    if (eq > from && eq < amp) {
      String k = query.substring(from, eq);
      if (k == key) {
        return query.substring(eq + 1, amp);
      }
    }
    from = amp + 1;
  }
  return "";
}

void sendHttp(EthernetClient &client, const char *status, const char *type, const String &body) {
  client.print("HTTP/1.1 ");
  client.println(status);
  client.print("Content-Type: ");
  client.println(type);
  client.println("Cache-Control: no-store");
  client.print("Content-Length: ");
  client.println(body.length());
  client.println("Connection: close");
  client.println();
  client.print(body);
}

void sendStateJson(EthernetClient &client) {
  String ip = Ethernet.localIP().toString();
  if (ip == "0.0.0.0") {
    ip = "link-down";
  }

  String json;
  json.reserve(520);
  json += "{";
  json += "\"swr\":" + String(currentSwr, 3);
  json += ",\"frequencyMHz\":" + String(currentFrequencyMHz, 3);
  json += ",\"lengthDipoleMM\":" + String(targetLengthDipoleMM, 1);
  json += ",\"lengthReflectorMM\":" + String(targetLengthReflectorMM, 1);
  json += ",\"ethernetIp\":\"" + ip + "\"";
  json += ",\"autoTune\":" + boolJSON(autoTuneEnabled);
  json += ",\"motors\":[{";
  json += "\"position\":" + String(axisDipole.position());
  json += ",\"target\":" + String(axisDipole.target());
  json += ",\"limitMin\":" + boolJSON(axisDipole.limitMinTriggered());
  json += ",\"limitMax\":" + boolJSON(axisDipole.limitMaxTriggered());
  json += "},{";
  json += "\"position\":" + String(axisReflector.position());
  json += ",\"target\":" + String(axisReflector.target());
  json += ",\"limitMin\":" + boolJSON(axisReflector.limitMinTriggered());
  json += ",\"limitMax\":" + boolJSON(axisReflector.limitMaxTriggered());
  json += "}]}";

  sendHttp(client, "200 OK", "application/json", json);
}

bool startsWith(const String &v, const char *prefix) {
  return v.startsWith(prefix);
}

void handleApi(EthernetClient &client, const String &path, const String &query) {
  if (path == "/api/state") {
    sendStateJson(client);
    return;
  }

  if (path == "/api/frequency") {
    String f = getArg(query, "mhz");
    if (f.length() == 0) {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"missing mhz\"}");
      return;
    }
    applyFrequencyModel(f.toFloat());
    sendHttp(client, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/auto") {
    String on = getArg(query, "on");
    autoTuneEnabled = (on == "1");
    if (autoTuneEnabled) {
      autoBestSwr = currentSwr;
    }
    sendHttp(client, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/jog") {
    String m = getArg(query, "motor");
    String d = getArg(query, "delta");
    if (m.length() == 0 || d.length() == 0) {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"missing params\"}");
      return;
    }
    int motor = m.toInt();
    long delta = d.toInt();
    if (motor == AXIS_DIPOLE) {
      axisDipole.jog(delta, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
    } else if (motor == AXIS_REFLECTOR) {
      axisReflector.jog(delta, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
    } else {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"invalid motor\"}");
      return;
    }
    sendHttp(client, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/home") {
    String m = getArg(query, "motor");
    if (m.length() == 0) {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"missing motor\"}");
      return;
    }
    int motor = m.toInt();
    if (motor == AXIS_DIPOLE) {
      axisDipole.home();
    } else if (motor == AXIS_REFLECTOR) {
      axisReflector.home();
    } else {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"invalid motor\"}");
      return;
    }
    sendHttp(client, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/stop") {
    String m = getArg(query, "motor");
    if (m.length() == 0) {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"missing motor\"}");
      return;
    }
    int motor = m.toInt();
    if (motor == AXIS_DIPOLE) {
      axisDipole.stop();
    } else if (motor == AXIS_REFLECTOR) {
      axisReflector.stop();
    } else {
      sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"invalid motor\"}");
      return;
    }
    sendHttp(client, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  sendHttp(client, "404 Not Found", "application/json", "{\"error\":\"not found\"}");
}

void handleHttpClient() {
  EthernetClient client = ethServer.available();
  if (!client) {
    return;
  }

  String requestLine;
  uint32_t start = millis();
  while (client.connected() && millis() - start < 1200) {
    if (!client.available()) {
      delay(1);
      continue;
    }
    requestLine = client.readStringUntil('\n');
    break;
  }
  requestLine.trim();

  while (client.connected() && client.available()) {
    String discard = client.readStringUntil('\n');
    if (discard == "\r" || discard.length() == 0) {
      break;
    }
  }

  int firstSpace = requestLine.indexOf(' ');
  int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
  if (firstSpace < 0 || secondSpace < 0) {
    sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"bad request\"}");
    client.stop();
    return;
  }

  String target = requestLine.substring(firstSpace + 1, secondSpace);
  String path = target;
  String query;
  int q = target.indexOf('?');
  if (q >= 0) {
    path = target.substring(0, q);
    query = target.substring(q + 1);
  }

  if (path == "/") {
    sendHttp(client, "200 OK", "text/html", String(INDEX_HTML));
  } else if (startsWith(path, "/api/")) {
    handleApi(client, path, query);
  } else {
    sendHttp(client, "404 Not Found", "text/plain", "Not found");
  }

  delay(1);
  client.stop();
}

void setupEthernet() {
  SPI.begin(ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);

  if (ETH_RST_PIN >= 0) {
    pinMode(ETH_RST_PIN, OUTPUT);
    digitalWrite(ETH_RST_PIN, LOW);
    delay(30);
    digitalWrite(ETH_RST_PIN, HIGH);
    delay(160);
  }

  int dhcpOk = Ethernet.begin(ETH_MAC, 5000, 1000);
  if (dhcpOk == 0) {
    Ethernet.begin(ETH_MAC, fallbackIp, fallbackDns, fallbackGw, fallbackMask);
  }

  ethServer.begin();

  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());
}

} // namespace

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);

  axisDipole.begin();
  axisReflector.begin();

  setupEthernet();
  applyFrequencyModel(currentFrequencyMHz);
  Serial.println("URBAN BEAM controller ready.");
}

void loop() {
  axisDipole.update(MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  axisReflector.update(MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);

  handleHttpClient();
  Ethernet.maintain();

  uint32_t now = millis();
  if (now - lastStateSampleMs >= STATE_SAMPLE_MS) {
    lastStateSampleMs = now;
    currentSwr = computeSWR();
  }

  updateAutoTuning();
}