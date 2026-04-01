// ============================================================================
// URBAN BEAM – Inline SWR Meter + Stepper Controller
// ESP32-S3 + W5500 Ethernet
//
// Architecture:
//   [TX] --BNC IN--> [coupleur directionnel] --BNC OUT--> [antenne]
//                          |
//                   [carte mesure SWR]  (PCB séparé, à côté de l'ESP)
//                          |
//                    GPI: ADC_SWR (tension proportionnelle au ROS)
//                          |
//                   [ESP32-S3  Waveshare]
//                     |             |
//                GPO: step/dir   Ethernet web UI
//                     |
//              [boîtier drivers moteurs]  (externe)
//                     |           |
//               axe dipole   axe reflector
//
// Fréquence: saisie manuelle OU détection auto via compteur PCNT
//            (comparateur LM311 + prescaler 74HC4040 ÷256 → GPIO)
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <AccelStepper.h>
#include <driver/pcnt.h>

namespace {

// ── Pin definitions ──────────────────────────────────────────────────────────

// GPO: stepper control (vers boîtier drivers externe).
struct MotorPins {
  uint8_t step;
  uint8_t dir;
  uint8_t enable;
  uint8_t limitMin;
  uint8_t limitMax;
};

enum AxisId : uint8_t { AXIS_DIPOLE = 0, AXIS_REFLECTOR = 1 };

MotorPins motorPins[2] = {
    {4, 5, 6, 7, 15},   // Dipole
    {16, 17, 18, 8, 3},  // Reflector
};

// GPI: SWR measurement – single analog input from measurement board.
// The board outputs a DC voltage proportional to VSWR (e.g. 0-3.3V → 1:1–∞).
constexpr uint8_t SWR_ADC_PIN = 1;

// GPI: Frequency counter input (after prescaler ÷256).
// On the measurement PCB: RF tap → LM311 comparator → 74HC4040 ÷256 → this pin.
constexpr uint8_t FREQ_COUNTER_PIN = 2;

// Ethernet (W5500 over SPI).
constexpr uint8_t ETH_MISO_PIN = 13;
constexpr uint8_t ETH_MOSI_PIN = 11;
constexpr uint8_t ETH_SCK_PIN  = 12;
constexpr uint8_t ETH_CS_PIN   = 10;
constexpr int8_t  ETH_RST_PIN  = 9;

// ── Motor parameters ─────────────────────────────────────────────────────────

constexpr float MAX_SPEED_STEPS_S = 1800.0f;
constexpr float ACCEL_STEPS_S2    = 1200.0f;
constexpr long  MAX_TRAVEL_STEPS[2] = {120000, 120000};

constexpr float DIPOLE_REF_LENGTH_MM    = 9960.0f;
constexpr float REFLECTOR_REF_LENGTH_MM = 9600.0f;
constexpr long  DIPOLE_REF_STEPS        = 42000;
constexpr long  REFLECTOR_REF_STEPS     = 45000;
constexpr float DIPOLE_STEPS_PER_MM     = -3.8f;
constexpr float REFLECTOR_STEPS_PER_MM  = -3.5f;

// ── Auto-tune ────────────────────────────────────────────────────────────────

constexpr float    SWR_TARGET           = 1.30f;
constexpr float    SWR_HIGH_MARGIN      = 0.10f;
constexpr float    SWR_IMPROVEMENT_HYST = 0.02f;
constexpr long     AUTO_MIN_STEP        = 8;
constexpr long     AUTO_MAX_STEP        = 240;
constexpr uint32_t AUTO_UPDATE_MS       = 900;

// ── ADC ──────────────────────────────────────────────────────────────────────

constexpr float ADC_VREF = 3.30f;
constexpr int   ADC_MAX  = 4095;

// SWR conversion: the measurement board outputs a voltage.
// Linear mapping: 0V → SWR 1.0, 3.3V → SWR_VOLTAGE_MAX_SWR.
// Adjust after real calibration.
constexpr float SWR_VOLTAGE_MAX_SWR = 10.0f;

constexpr uint32_t SWR_SAMPLE_MS = 300;

// ── Frequency counter ────────────────────────────────────────────────────────
// The 74HC4040 divides the RF frequency by 256.
// PCNT counts rising edges over a gate period.
// Measured frequency = (count * 256) / gate_seconds.

constexpr uint16_t FREQ_PRESCALER   = 256;
constexpr uint32_t FREQ_GATE_MS     = 100;   // 100ms gate → 10 Hz resolution × 256
constexpr pcnt_unit_t PCNT_UNIT     = PCNT_UNIT_0;

// ── Ethernet ─────────────────────────────────────────────────────────────────

byte ETH_MAC[] = {0x02, 0x55, 0x42, 0x10, 0x20, 0x33};
IPAddress fallbackIp(192, 168, 1, 70);
IPAddress fallbackDns(192, 168, 1, 1);
IPAddress fallbackGw(192, 168, 1, 1);
IPAddress fallbackMask(255, 255, 255, 0);
EthernetServer ethServer(80);

// ── Axis class ───────────────────────────────────────────────────────────────

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
    long cur = stepper_.currentPosition();
    if (cur <= 0 && stepper_.distanceToGo() < 0) {
      stepper_.setCurrentPosition(0); stepper_.moveTo(0); target_ = 0; return;
    }
    if (cur >= maxTravel && stepper_.distanceToGo() > 0) {
      stepper_.setCurrentPosition(maxTravel); stepper_.moveTo(maxTravel); target_ = maxTravel; return;
    }
    if (stepper_.distanceToGo() < 0 && limitMinTriggered()) {
      stepper_.setCurrentPosition(0); stepper_.moveTo(0); target_ = 0; return;
    }
    if (stepper_.distanceToGo() > 0 && limitMaxTriggered()) {
      long lock = stepper_.currentPosition();
      stepper_.setCurrentPosition(lock); stepper_.moveTo(lock); target_ = lock; return;
    }
    stepper_.run();
  }

  void home()  { homing_ = true; stepper_.setSpeed(-700.0f); }
  void stop()  { homing_ = false; stepper_.stop(); target_ = stepper_.currentPosition(); }

  void moveTo(long abs, long maxT) {
    long c = constrain(abs, 0L, maxT);
    if (limitMinTriggered() && c < stepper_.currentPosition()) c = stepper_.currentPosition();
    if (limitMaxTriggered() && c > stepper_.currentPosition()) c = stepper_.currentPosition();
    target_ = c; stepper_.moveTo(c);
  }

  void jog(long delta, long maxT) { moveTo(stepper_.currentPosition() + delta, maxT); }

  long position()  { return stepper_.currentPosition(); }
  long target() const { return target_; }
  bool busy()      { return homing_ || stepper_.distanceToGo() != 0; }
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

// ── Global state ─────────────────────────────────────────────────────────────

float    currentFrequencyMHz     = 7.10f;
float    detectedFrequencyMHz    = 0.0f;   // auto-detected via PCNT
float    currentSwr              = 99.0f;
bool     freqAutoMode            = false;   // true = use detected frequency
float    autoBestSwr             = 99.0f;
bool     autoTuneEnabled         = false;
uint8_t  autoAxisRoundRobin      = 0;
int8_t   autoDirection[2]        = {1, 1};
long     autoStep[2]             = {60, 60};
uint32_t lastSwrSampleMs         = 0;
uint32_t lastAutoUpdateMs        = 0;
uint32_t lastFreqMeasureMs       = 0;
float    targetLengthDipoleMM    = 0.0f;
float    targetLengthReflectorMM = 0.0f;

// ── Frequency counter (PCNT) ─────────────────────────────────────────────────

void pcntInit() {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = FREQ_COUNTER_PIN;
  cfg.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = PCNT_UNIT;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DIS;
  cfg.lctrl_mode     = PCNT_MODE_KEEP;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.counter_h_lim  = 32767;
  cfg.counter_l_lim  = 0;

  pcnt_unit_config(&cfg);
  pcnt_counter_pause(PCNT_UNIT);
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
}

// Returns frequency in MHz, or 0 if no signal.
float pcntMeasureFreqMHz() {
  pcnt_counter_clear(PCNT_UNIT);
  pcnt_counter_resume(PCNT_UNIT);
  delay(FREQ_GATE_MS);
  pcnt_counter_pause(PCNT_UNIT);

  int16_t count = 0;
  pcnt_get_counter_value(PCNT_UNIT, &count);

  if (count <= 1) return 0.0f;

  // freq_Hz = count * prescaler / gate_seconds
  double freqHz = static_cast<double>(count) * FREQ_PRESCALER /
                  (static_cast<double>(FREQ_GATE_MS) / 1000.0);
  return static_cast<float>(freqHz / 1e6);
}

// ── SWR measurement ──────────────────────────────────────────────────────────

float readSWR() {
  uint32_t sum = 0;
  for (int i = 0; i < 16; ++i) {
    sum += analogRead(SWR_ADC_PIN);
    delayMicroseconds(100);
  }
  float volts = (static_cast<float>(sum) / 16.0f / static_cast<float>(ADC_MAX)) * ADC_VREF;
  // Linear mapping: 0V→SWR 1.0, 3.3V→SWR_VOLTAGE_MAX_SWR.
  // Adjust this formula to match your measurement board's output characteristic.
  float swr = 1.0f + (volts / ADC_VREF) * (SWR_VOLTAGE_MAX_SWR - 1.0f);
  return max(swr, 1.0f);
}

// ── Antenna length formulas ──────────────────────────────────────────────────

float quarterWaveDipoleMM(float fMHz) {
  return ((300.0f / fMHz) * 0.95f) / 4.0f * 1000.0f;
}

float quarterWaveReflectorMM(float fMHz) {
  return ((300.0f / fMHz) * 0.89f) / 4.0f * 1000.0f;
}

long dipoleStepsFromMM(float mm) {
  float off = mm - DIPOLE_REF_LENGTH_MM;
  return constrain(static_cast<long>(DIPOLE_REF_STEPS + off * DIPOLE_STEPS_PER_MM),
                   0L, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
}

long reflectorStepsFromMM(float mm) {
  float off = mm - REFLECTOR_REF_LENGTH_MM;
  return constrain(static_cast<long>(REFLECTOR_REF_STEPS + off * REFLECTOR_STEPS_PER_MM),
                   0L, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
}

// ── Apply frequency → move motors ────────────────────────────────────────────

void applyFrequency(float freqMHz) {
  currentFrequencyMHz = constrain(freqMHz, 1.0f, 60.0f);
  targetLengthDipoleMM    = quarterWaveDipoleMM(currentFrequencyMHz);
  targetLengthReflectorMM = quarterWaveReflectorMM(currentFrequencyMHz);

  axisDipole.moveTo(dipoleStepsFromMM(targetLengthDipoleMM),     MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  axisReflector.moveTo(reflectorStepsFromMM(targetLengthReflectorMM), MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);

  autoBestSwr = 99.0f;
  autoStep[AXIS_DIPOLE] = 60;   autoStep[AXIS_REFLECTOR] = 60;
  autoDirection[AXIS_DIPOLE] = 1; autoDirection[AXIS_REFLECTOR] = 1;
}

// ── Auto-tune (tâtonnement) ──────────────────────────────────────────────────

void updateAutoTuning() {
  if (!autoTuneEnabled) return;
  if (axisDipole.busy() || axisReflector.busy()) return;

  uint32_t now = millis();
  if (now - lastAutoUpdateMs < AUTO_UPDATE_MS) return;
  lastAutoUpdateMs = now;

  if (currentSwr <= SWR_TARGET + SWR_HIGH_MARGIN) return;

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
  if (axis == AXIS_DIPOLE)
    axisDipole.jog(delta, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  else
    axisReflector.jog(delta, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
}

// ── Web UI ───────────────────────────────────────────────────────────────────

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="fr"><head>
<meta charset="utf-8"/><meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>URBAN BEAM – SWR Controller</title>
<style>
:root{--bg1:#0f172a;--bg2:#1e293b;--card:rgba(255,255,255,.09);--ok:#34d399;--warn:#f59e0b;--bad:#ef4444;--txt:#e2e8f0;--muted:#94a3b8;--accent:#22d3ee}
*{box-sizing:border-box}
body{margin:0;font-family:"IBM Plex Sans","Segoe UI",sans-serif;color:var(--txt);
  background:radial-gradient(circle at 10% 10%,#1d4ed8 0%,transparent 35%),
             radial-gradient(circle at 90% 20%,#0ea5e9 0%,transparent 30%),
             linear-gradient(165deg,var(--bg1),var(--bg2));min-height:100vh;padding:16px}
.g{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:12px}
.c{background:var(--card);border:1px solid rgba(255,255,255,.13);border-radius:14px;padding:14px;backdrop-filter:blur(8px)}
h1{margin:0 0 10px;font-size:1.3rem}
h2{margin:0 0 6px;font-size:.95rem;color:#f8fafc}
.r{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin-top:6px}
.m{color:var(--muted);font-size:.85rem}
.badge{font-weight:700;color:#020617;background:#67e8f9;border-radius:999px;padding:3px 9px;font-size:.82rem}
button{background:rgba(15,23,42,.75);color:var(--txt);border:1px solid rgba(148,163,184,.5);border-radius:8px;padding:6px 10px;cursor:pointer;font-size:.82rem}
button:hover{border-color:var(--accent)}
button.active{border-color:var(--ok);background:rgba(52,211,153,.15)}
select,input{padding:5px 8px;border-radius:8px;border:1px solid rgba(148,163,184,.5);background:rgba(2,6,23,.7);color:var(--txt);font-size:.85rem}
input[type=number]{max-width:120px}
.swr-big{font-size:2.8rem;font-weight:800;text-align:center;margin:10px 0;letter-spacing:-.02em}
.swr-ok{color:var(--ok)}.swr-warn{color:var(--warn)}.swr-bad{color:var(--bad)}
.freq-big{font-size:1.4rem;font-weight:600;text-align:center;color:var(--accent)}
.info-row{display:flex;justify-content:space-between;padding:2px 0}
.info-row .label{color:var(--muted);font-size:.82rem}
.info-row .val{font-weight:600;font-size:.88rem}
</style></head><body>
<h1>URBAN BEAM – Controleur antenne HF</h1>
<div class="g">

<!-- SWR display -->
<section class="c">
  <h2>ROS / SWR</h2>
  <div id="swr" class="swr-big swr-warn">--</div>
  <div class="info-row"><span class="label">Ethernet</span><span class="val" id="ip">--</span></div>
</section>

<!-- Frequency -->
<section class="c">
  <h2>Frequence</h2>
  <div id="freqDisplay" class="freq-big">-- MHz</div>
  <div class="r">
    <button id="btnManual" class="active" onclick="setMode('manual')">Manuel</button>
    <button id="btnAuto" onclick="setMode('auto')">Auto (compteur)</button>
  </div>
  <div class="r" id="manualRow">
    <input id="freqIn" type="number" min="1" max="55" step="0.001" value="7.100"/>
    <label>MHz</label>
    <button onclick="applyFreq()">Appliquer</button>
  </div>
  <div class="r">
    <select id="bandSel" onchange="pickBand()">
      <option value="">-- Bande --</option>
      <option value="1.900">160m (1.9)</option>
      <option value="3.650">80m (3.65)</option>
      <option value="7.100" selected>40m (7.1)</option>
      <option value="10.125">30m (10.125)</option>
      <option value="14.175">20m (14.175)</option>
      <option value="18.118">17m (18.118)</option>
      <option value="21.225">15m (21.225)</option>
      <option value="24.940">12m (24.94)</option>
      <option value="28.500">10m (28.5)</option>
    </select>
  </div>
  <div class="m" style="margin-top:8px">Freq detectee: <b id="detFreq">--</b> MHz</div>
</section>

<!-- Antenna lengths -->
<section class="c">
  <h2>Longueurs antenne calculees</h2>
  <div class="info-row"><span class="label">Dipole 1/4&lambda;</span><span class="val" id="dipMM">-- mm</span></div>
  <div class="info-row"><span class="label">Reflector 1/4&lambda;</span><span class="val" id="refMM">-- mm</span></div>
  <div class="r"><span class="badge">Formule: (300/Fe &times; k) / 4</span></div>
</section>

<!-- Auto-tune -->
<section class="c">
  <h2>Ajustement auto (tatonnement)</h2>
  <div class="r">
    <button onclick="autoTune(1)">Activer</button>
    <button onclick="autoTune(0)">Stop</button>
  </div>
  <div class="m">Etat: <b id="autoSt">--</b></div>
  <div class="r"><span class="badge">Objectif SWR &le; 1.30</span></div>
</section>

<!-- Motor: Dipole -->
<section class="c">
  <h2>Axe Dipole</h2>
  <div class="info-row"><span class="label">Position</span><span class="val" id="m1pos">--</span></div>
  <div class="info-row"><span class="label">Fins de course</span><span class="val">min:<span id="m1min">--</span> max:<span id="m1max">--</span></span></div>
  <div class="r">
    <button onclick="jog(0,-500)">-500</button>
    <button onclick="jog(0,-100)">-100</button>
    <button onclick="jog(0,100)">+100</button>
    <button onclick="jog(0,500)">+500</button>
    <button onclick="home(0)">Home</button>
    <button onclick="stop(0)">Stop</button>
  </div>
</section>

<!-- Motor: Reflector -->
<section class="c">
  <h2>Axe Reflector</h2>
  <div class="info-row"><span class="label">Position</span><span class="val" id="m2pos">--</span></div>
  <div class="info-row"><span class="label">Fins de course</span><span class="val">min:<span id="m2min">--</span> max:<span id="m2max">--</span></span></div>
  <div class="r">
    <button onclick="jog(1,-500)">-500</button>
    <button onclick="jog(1,-100)">-100</button>
    <button onclick="jog(1,100)">+100</button>
    <button onclick="jog(1,500)">+500</button>
    <button onclick="home(1)">Home</button>
    <button onclick="stop(1)">Stop</button>
  </div>
</section>

</div>
<script>
function api(p){return fetch(p,{cache:'no-store'}).then(r=>r.json())}
function applyFreq(){
  const f=parseFloat(document.getElementById('freqIn').value||'7.1');
  fetch('/api/frequency?mhz='+f);
}
function pickBand(){
  const v=document.getElementById('bandSel').value;
  if(v){document.getElementById('freqIn').value=v;applyFreq()}
}
function setMode(m){
  fetch('/api/freqmode?mode='+m);
  document.getElementById('btnManual').className=m==='manual'?'active':'';
  document.getElementById('btnAuto').className=m==='auto'?'active':'';
}
function autoTune(on){fetch('/api/auto?on='+on)}
function jog(m,d){fetch('/api/jog?motor='+m+'&delta='+d)}
function home(m){fetch('/api/home?motor='+m)}
function stop(m){fetch('/api/stop?motor='+m)}

function swrClass(v){return v<=1.5?'swr-ok':v<=3?'swr-warn':'swr-bad'}

async function poll(){
  try{
    const s=await api('/api/state');
    const el=document.getElementById('swr');
    el.textContent=s.swr.toFixed(2);
    el.className='swr-big '+swrClass(s.swr);
    document.getElementById('freqDisplay').textContent=s.frequencyMHz.toFixed(3)+' MHz';
    document.getElementById('detFreq').textContent=s.detectedMHz>0?s.detectedMHz.toFixed(3):'aucun signal';
    document.getElementById('ip').textContent=s.ip;
    document.getElementById('dipMM').textContent=s.dipoleMM.toFixed(1)+' mm';
    document.getElementById('refMM').textContent=s.reflectorMM.toFixed(1)+' mm';
    document.getElementById('autoSt').textContent=s.autoTune?'ACTIF':'MANUEL';
    document.getElementById('m1pos').textContent=s.m[0].p;
    document.getElementById('m2pos').textContent=s.m[1].p;
    document.getElementById('m1min').textContent=s.m[0].lMin?'ON':'off';
    document.getElementById('m1max').textContent=s.m[0].lMax?'ON':'off';
    document.getElementById('m2min').textContent=s.m[1].lMin?'ON':'off';
    document.getElementById('m2max').textContent=s.m[1].lMax?'ON':'off';
    document.getElementById('btnManual').className=s.freqAuto?'':'active';
    document.getElementById('btnAuto').className=s.freqAuto?'active':'';
  }catch(_){}
}
setInterval(poll,500);poll();
</script></body></html>
)HTML";

// ── HTTP helpers ─────────────────────────────────────────────────────────────

String boolJSON(bool v) { return v ? "true" : "false"; }

String getArg(const String &query, const String &key) {
  int from = 0;
  while (from < static_cast<int>(query.length())) {
    int amp = query.indexOf('&', from);
    if (amp < 0) amp = query.length();
    int eq = query.indexOf('=', from);
    if (eq > from && eq < amp) {
      if (query.substring(from, eq) == key) return query.substring(eq + 1, amp);
    }
    from = amp + 1;
  }
  return "";
}

void sendHttp(EthernetClient &c, const char *status, const char *type, const String &body) {
  c.print("HTTP/1.1 "); c.println(status);
  c.print("Content-Type: "); c.println(type);
  c.println("Cache-Control: no-store");
  c.print("Content-Length: "); c.println(body.length());
  c.println("Connection: close");
  c.println();
  c.print(body);
}

void sendStateJson(EthernetClient &c) {
  String ip = Ethernet.localIP().toString();
  if (ip == "0.0.0.0") ip = "link-down";

  String j;
  j.reserve(500);
  j += "{\"swr\":" + String(currentSwr, 2);
  j += ",\"frequencyMHz\":" + String(currentFrequencyMHz, 3);
  j += ",\"detectedMHz\":" + String(detectedFrequencyMHz, 3);
  j += ",\"freqAuto\":" + boolJSON(freqAutoMode);
  j += ",\"dipoleMM\":" + String(targetLengthDipoleMM, 1);
  j += ",\"reflectorMM\":" + String(targetLengthReflectorMM, 1);
  j += ",\"ip\":\"" + ip + "\"";
  j += ",\"autoTune\":" + boolJSON(autoTuneEnabled);
  j += ",\"m\":[{\"p\":" + String(axisDipole.position());
  j += ",\"t\":" + String(axisDipole.target());
  j += ",\"lMin\":" + boolJSON(axisDipole.limitMinTriggered());
  j += ",\"lMax\":" + boolJSON(axisDipole.limitMaxTriggered());
  j += "},{\"p\":" + String(axisReflector.position());
  j += ",\"t\":" + String(axisReflector.target());
  j += ",\"lMin\":" + boolJSON(axisReflector.limitMinTriggered());
  j += ",\"lMax\":" + boolJSON(axisReflector.limitMaxTriggered());
  j += "}]}";
  sendHttp(c, "200 OK", "application/json", j);
}

// ── API router ───────────────────────────────────────────────────────────────

void handleApi(EthernetClient &c, const String &path, const String &q) {

  if (path == "/api/state") { sendStateJson(c); return; }

  if (path == "/api/frequency") {
    String f = getArg(q, "mhz");
    if (f.length() == 0) { sendHttp(c, "400 Bad Request", "application/json", "{\"error\":\"missing mhz\"}"); return; }
    freqAutoMode = false;
    applyFrequency(f.toFloat());
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/freqmode") {
    String mode = getArg(q, "mode");
    freqAutoMode = (mode == "auto");
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/auto") {
    autoTuneEnabled = (getArg(q, "on") == "1");
    if (autoTuneEnabled) autoBestSwr = currentSwr;
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/jog") {
    int motor = getArg(q, "motor").toInt();
    long delta = getArg(q, "delta").toInt();
    if (motor == AXIS_DIPOLE) axisDipole.jog(delta, MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
    else if (motor == AXIS_REFLECTOR) axisReflector.jog(delta, MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);
    else { sendHttp(c, "400 Bad Request", "application/json", "{\"error\":\"bad motor\"}"); return; }
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/home") {
    int motor = getArg(q, "motor").toInt();
    if (motor == AXIS_DIPOLE) axisDipole.home();
    else if (motor == AXIS_REFLECTOR) axisReflector.home();
    else { sendHttp(c, "400 Bad Request", "application/json", "{\"error\":\"bad motor\"}"); return; }
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  if (path == "/api/stop") {
    int motor = getArg(q, "motor").toInt();
    if (motor == AXIS_DIPOLE) axisDipole.stop();
    else if (motor == AXIS_REFLECTOR) axisReflector.stop();
    else { sendHttp(c, "400 Bad Request", "application/json", "{\"error\":\"bad motor\"}"); return; }
    sendHttp(c, "200 OK", "application/json", "{\"ok\":true}");
    return;
  }

  sendHttp(c, "404 Not Found", "application/json", "{\"error\":\"not found\"}");
}

// ── HTTP client handler ──────────────────────────────────────────────────────

void handleHttpClient() {
  EthernetClient client = ethServer.available();
  if (!client) return;

  String requestLine;
  uint32_t t0 = millis();
  while (client.connected() && millis() - t0 < 1200) {
    if (!client.available()) { delay(1); continue; }
    requestLine = client.readStringUntil('\n');
    break;
  }
  requestLine.trim();

  while (client.connected() && client.available()) {
    String d = client.readStringUntil('\n');
    if (d == "\r" || d.length() == 0) break;
  }

  int s1 = requestLine.indexOf(' ');
  int s2 = requestLine.indexOf(' ', s1 + 1);
  if (s1 < 0 || s2 < 0) {
    sendHttp(client, "400 Bad Request", "application/json", "{\"error\":\"bad\"}");
    client.stop(); return;
  }

  String target = requestLine.substring(s1 + 1, s2);
  String path = target, query;
  int qi = target.indexOf('?');
  if (qi >= 0) { path = target.substring(0, qi); query = target.substring(qi + 1); }

  if (path == "/")
    sendHttp(client, "200 OK", "text/html", String(INDEX_HTML));
  else if (path.startsWith("/api/"))
    handleApi(client, path, query);
  else
    sendHttp(client, "404 Not Found", "text/plain", "Not found");

  delay(1);
  client.stop();
}

// ── Ethernet setup ───────────────────────────────────────────────────────────

void setupEthernet() {
  SPI.begin(ETH_SCK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN, ETH_CS_PIN);
  Ethernet.init(ETH_CS_PIN);
  if (ETH_RST_PIN >= 0) {
    pinMode(ETH_RST_PIN, OUTPUT);
    digitalWrite(ETH_RST_PIN, LOW);  delay(30);
    digitalWrite(ETH_RST_PIN, HIGH); delay(160);
  }
  if (Ethernet.begin(ETH_MAC, 5000, 1000) == 0)
    Ethernet.begin(ETH_MAC, fallbackIp, fallbackDns, fallbackGw, fallbackMask);
  ethServer.begin();
  Serial.print("IP: "); Serial.println(Ethernet.localIP());
}

} // namespace

// ── setup / loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(300);
  analogReadResolution(12);

  pcntInit();

  axisDipole.begin();
  axisReflector.begin();
  setupEthernet();

  applyFrequency(currentFrequencyMHz);
  Serial.println("URBAN BEAM SWR controller ready.");
}

void loop() {
  axisDipole.update(MAX_TRAVEL_STEPS[AXIS_DIPOLE]);
  axisReflector.update(MAX_TRAVEL_STEPS[AXIS_REFLECTOR]);

  handleHttpClient();
  Ethernet.maintain();

  uint32_t now = millis();

  // Periodic SWR reading.
  if (now - lastSwrSampleMs >= SWR_SAMPLE_MS) {
    lastSwrSampleMs = now;
    currentSwr = readSWR();
  }

  // Periodic frequency detection (every 500ms, non-blocking-ish).
  if (now - lastFreqMeasureMs >= 500) {
    lastFreqMeasureMs = now;
    detectedFrequencyMHz = pcntMeasureFreqMHz();
    // If in auto mode and a valid signal is detected, apply it.
    if (freqAutoMode && detectedFrequencyMHz >= 1.0f) {
      // Only re-apply if frequency changed significantly (>50 kHz).
      if (fabsf(detectedFrequencyMHz - currentFrequencyMHz) > 0.050f) {
        applyFrequency(detectedFrequencyMHz);
      }
    }
  }

  updateAutoTuning();
}