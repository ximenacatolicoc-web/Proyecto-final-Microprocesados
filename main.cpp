#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MAX30105.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ---------------- RED WI-FI (ACCESS POINT) ----------------
const char* AP_SSID = "ESP32_Pulse_Sensor";
const char* AP_PASS = "12345678"; // 
WebServer server(80);

// ---------------- PINES ----------------
#define SENSOR_SDA 21
#define SENSOR_SCL 22

#define OLED_SDA   25
#define OLED_SCL   26
#define OLED_ADDR  0x3C
#define SCREEN_W   128
#define SCREEN_H   64

#define SW420_PIN  32
#define BUZZER_PIN 27
#define SD_CS      5

// ---------------- OBJETOS Y BUSES ----------------
MAX30105 sensor;
TwoWire BusOLED = TwoWire(1);
Adafruit_SSD1306 oled(SCREEN_W, SCREEN_H, &BusOLED, -1);

bool oledOk = false;
bool sdOk = false;

// ---------------- FILTRADO, FC (BPM) Y SPO2 ----------------
const float SAMPLE_RATE_HZ = 100.0f;

float irBaseline = 0.0f;
float redBaseline = 0.0f;
float signalFiltered = 0.0f;
float amplitude = 0.0f;
float threshold = 0.0f;
bool initialized = false;

float irAcMax = -1e9f, irAcMin = 1e9f;
float redAcMax = -1e9f, redAcMin = 1e9f;
unsigned long acWindowSamples = 0;
const unsigned long AC_WINDOW_SIZE = 100;

float sample0 = 0.0f;
float sample1 = 0.0f;
float sample2 = 0.0f;

unsigned long sampleCounter = 0;
unsigned long lastBeatSample = 0;

float bpm = 0.0f;
float bpmAverage = 0.0f;
float spo2Est = 0.0f;
long currentIR = 0;
long currentRED = 0;

const byte RATE_SIZE = 4;
float rates[RATE_SIZE] = {0};
byte rateSpot = 0;
byte validBeats = 0;

// ---------------- SW-420, ALARMA UNICA Y BUZZER ----------------
bool vibrationActive = false;
unsigned long lastVibrationMs = 0;
const unsigned long VIBRATION_HOLD_MS = 600;

float spo2Threshold = 92.0f;       // Umbral configurable de alarma SpO2 (%)
bool buzzerManualTest = false;     // Prueba manual de buzzer desde la web
bool buzzerMuted = false;          // Silenciar alarma automatica desde la web
bool sdLoggingEnabled = true;      // Iniciar/pausar registro en SD desde la web
unsigned long manualBuzzerUntil = 0;

bool alertActive = false;
const char* alertReason = "NONE";
bool buzzerState = false;
unsigned long lastBuzzerToggle = 0;

// ---------------- INTERVALOS (NO BLOQUEANTES) ----------------
unsigned long lastOledUpdate = 0;
unsigned long lastSdWrite = 0;
unsigned long lastSerialPrint = 0;

const unsigned long OLED_INTERVAL   = 250;
const unsigned long SD_INTERVAL     = 1000;
const unsigned long SERIAL_INTERVAL = 500;

// ---------------- INTERFAZ WEB BIDIRECCIONAL ----------------
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>ESP32 Pulsoximetro</title>
<style>
  body { font-family: Arial, sans-serif; background: #10151c; color: #eef3f8; margin: 0; padding: 16px; }
  h1 { font-size: 1.3rem; margin: 0 0 14px 0; }
  .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 10px; margin-bottom: 16px; max-width: 760px; }
  .card { background: #1b2430; border-radius: 10px; padding: 12px; border: 1px solid #2b384a; }
  .label { font-size: 0.8rem; color: #9fb0c7; }
  .val { font-size: 1.5rem; font-weight: bold; margin-top: 4px; }
  .alert { border-color: #e04f5f; background: #331920; color: #ff9aa5; }
  canvas { width: 100%; max-width: 760px; height: 220px; background: #161e28; border-radius: 10px; border: 1px solid #2b384a; display: block; }
  .controls { display: flex; flex-wrap: wrap; gap: 10px; align-items: center; margin-top: 10px; }
  input { width: 78px; padding: 6px; border-radius: 6px; border: 1px solid #3b4d66; background: #10151c; color: #eef3f8; }
  button { padding: 7px 12px; border-radius: 6px; border: none; color: #fff; cursor: pointer; font-weight: bold; }
</style>
</head>
<body>
  <h1>Monitor Biomedico Local (ESP32 AP)</h1>
  <div class="grid">
    <div class="card"><div class="label">FC (BPM)</div><div class="val" id="bpm">--</div></div>
    <div class="card"><div class="label">SpO2 (%)</div><div class="val" id="spo2">--</div></div>
    <div class="card"><div class="label">SW-420</div><div class="val" id="vib">--</div></div>
    <div class="card"><div class="label">MicroSD</div><div class="val" id="sd">--</div></div>
    <div class="card" id="alertCard"><div class="label">Alarma</div><div class="val" id="alert">NONE</div></div>
  </div>

  <canvas id="chart" width="760" height="220"></canvas>

  <div class="card" style="margin-top:14px; max-width:760px;">
    <div class="label">Configuracion y Comando Remoto</div>
    <div class="controls">
      <label>Umbral SpO2 (%):
        <input type="number" id="spo2Input" min="75" max="99" step="0.5" value="92.0">
      </label>
      <button style="background:#0284c7;" onclick="applyThreshold()">Aplicar Umbral</button>
      <button style="background:#d97706;" onclick="sendConfig({testBuzzer: 1})">Probar Buzzer</button>
      <button id="muteBtn" style="background:#475569;" onclick="toggleMute()">Silenciar: NO</button>
      <button id="sdBtn" style="background:#16a34a;" onclick="toggleSd()">Registro SD: ON</button>
    </div>
  </div>

<script>
const canvas = document.getElementById('chart');
const ctx = canvas.getContext('2d');
const historyLen = 120;
const waveData = new Array(historyLen).fill(0);

let currentMute = false;
let currentSdLog = true;
let thresholdSynced = false;

function drawWave() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.strokeStyle = '#38bdf8';
  ctx.lineWidth = 2;
  ctx.beginPath();
  const midY = canvas.height / 2;
  for (let i = 0; i < historyLen; i++) {
    const x = (i / (historyLen - 1)) * canvas.width;
    const y = midY - Math.max(-80, Math.min(80, waveData[i] * 0.4));
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

async function sendConfig(params) {
  const qs = new URLSearchParams(params).toString();
  const res = await fetch('/config?' + qs);
  const cfg = await res.json();
  if (cfg.spo2Min !== undefined) {
    document.getElementById('spo2Input').value = Number(cfg.spo2Min).toFixed(1);
    thresholdSynced = true;
  }
  pollData();
}

function applyThreshold() {
  const val = parseFloat(document.getElementById('spo2Input').value);
  if (!isNaN(val)) {
    sendConfig({ spo2Min: val });
  }
}

function toggleMute() {
  sendConfig({ mute: currentMute ? 0 : 1 });
}

function toggleSd() {
  sendConfig({ sdLog: currentSdLog ? 0 : 1 });
}

async function pollData() {
  try {
    const res = await fetch('/data');
    const d = await res.json();

    document.getElementById('bpm').textContent = d.finger ? d.bpm.toFixed(1) : '--';
    document.getElementById('spo2').textContent = (d.finger && d.spo2 > 0) ? d.spo2.toFixed(1) + '%' : '--';
    document.getElementById('vib').textContent = d.vib ? 'MOV' : 'QUIETO';
    document.getElementById('sd').textContent = !d.sd ? 'NO' : (d.sdLog ? 'OK' : 'PAUSA');

    if (!thresholdSynced && d.spo2Min !== undefined) {
      document.getElementById('spo2Input').value = Number(d.spo2Min).toFixed(1);
      thresholdSynced = true;
    }

    currentMute = !!d.mute;
    currentSdLog = !!d.sdLog;

    const muteBtn = document.getElementById('muteBtn');
    muteBtn.textContent = 'Silenciar: ' + (currentMute ? 'SI' : 'NO');
    muteBtn.style.background = currentMute ? '#b91c1c' : '#475569';

    const sdBtn = document.getElementById('sdBtn');
    sdBtn.textContent = 'Registro SD: ' + (currentSdLog ? 'ON' : 'OFF');
    sdBtn.style.background = currentSdLog ? '#16a34a' : '#64748b';

    const alertEl = document.getElementById('alert');
    const cardEl = document.getElementById('alertCard');
    alertEl.textContent = d.alert;
    if (d.alertActive) cardEl.classList.add('alert');
    else cardEl.classList.remove('alert');

    waveData.push(d.finger ? d.wave : 0);
    waveData.shift();
    drawWave();
  } catch (e) {}
}

setInterval(pollData, 150);
pollData();
</script>
</body>
</html>
)rawliteral";

void handleRoot() {
    server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() { //entrega el estado actual
    bool finger = (currentIR >= 15000); //deteccion de dedo en sensor 
    String json = "{";
    json += "\"finger\":" + String(finger ? "true" : "false") + ","; 
    json += "\"ir\":" + String(currentIR) + ",";
    json += "\"red\":" + String(currentRED) + ",";
    json += "\"wave\":" + String(signalFiltered, 2) + ",";
    json += "\"bpm\":" + String(bpmAverage, 1) + ",";
    json += "\"spo2\":" + String(spo2Est, 1) + ",";
    json += "\"spo2Min\":" + String(spo2Threshold, 1) + ",";
    json += "\"vib\":" + String(vibrationActive ? "true" : "false") + ",";
    json += "\"sd\":" + String(sdOk ? "true" : "false") + ",";
    json += "\"sdLog\":" + String(sdLoggingEnabled ? "true" : "false") + ",";
    json += "\"mute\":" + String(buzzerMuted ? "true" : "false") + ","; //configuracion de la alarma
    json += "\"alertActive\":" + String(alertActive ? "true" : "false") + ","; 
    json += "\"alert\":\"" + String(alertReason) + "\"";
    json += "}";
    server.send(200, "application/json", json);
}

void handleConfig() { //  procesa y actualiza los parametros del sistema
    if (server.hasArg("spo2Min")) { // umbral minimo de Spo2 
        float val = server.arg("spo2Min").toFloat();
        if (val >= 75.0f && val <= 99.0f) {
            spo2Threshold = val;
        }
    }
    if (server.hasArg("sdLog")) {
        sdLoggingEnabled = (server.arg("sdLog") == "1" || server.arg("sdLog") == "true");
    }
    if (server.hasArg("mute")) { // silenciar la alarma 
        buzzerMuted = (server.arg("mute") == "1" || server.arg("mute") == "true");
    }
    if (server.hasArg("testBuzzer")) { // probar la alarma 
        bool doTest = (server.arg("testBuzzer") == "1" || server.arg("testBuzzer") == "true");
        if (doTest) {
            buzzerManualTest = true;
            manualBuzzerUntil = millis() + 1500;
        } else {
            buzzerManualTest = false;
            manualBuzzerUntil = 0;
        }
    }

    String resp = "{\"ok\":true,\"spo2Min\":" + String(spo2Threshold, 1) +
                  ",\"sdLog\":" + String(sdLoggingEnabled ? "true" : "false") +
                  ",\"mute\":" + String(buzzerMuted ? "true" : "false") + "}";
    server.send(200, "application/json", resp);
}

void appendToSD(const char* path, const String& line) { //agragar datos en SD
    if (!sdOk) return;
    File file = SD.open(path, FILE_APPEND);
    if (file) {
        file.println(line);
        file.close();
    } else {
        Serial.println("Error al escribir en /registro.csv");
    }
}

void resetPulseState() { // reinicio de variables, cuando no hay dedo para evitar traer lecturas de datos anteriores 
    bpm = 0.0f;
    bpmAverage = 0.0f;
    spo2Est = 0.0f;
    validBeats = 0;
    rateSpot = 0;
    lastBeatSample = 0;
    acWindowSamples = 0;
    irAcMax = -1e9f;
    irAcMin = 1e9f;
    redAcMax = -1e9f;
    redAcMin = 1e9f;
}

void updateSpO2Estimate(float irAc, float redAc) { // estimacion porcentaje de saturacion de oxigeno
    if (irBaseline < 15000.0f || redBaseline < 1000.0f || irAc <= 1.0f) {
        return;
    }

    float ratioR = (redAc / redBaseline) / (irAc / irBaseline); //razon normalizada R 
    float spo2 = 104.0f - 17.0f * ratioR; // estima spo2
    if (spo2 > 100.0f) spo2 = 100.0f; //recorte si la señal esta por encima de 100.0f
    if (spo2 >= 70.0f && spo2 <= 100.0f) {  //suaviza la salida
        if (spo2Est == 0.0f) {
            spo2Est = spo2;
        } else {
            spo2Est = 0.8f * spo2Est + 0.2f * spo2;
        }
    }
}

void processSample(long ir, long red) { //procesamiento de la señal
    //guardar muestras 
    currentIR = ir; 
    currentRED = red;
    sampleCounter++;

    if (!initialized) { //inicializacion 
        irBaseline = (float)ir;
        redBaseline = (float)red;
        signalFiltered = 0.0f;
        initialized = true;
        return;
    }

    if (ir < 15000) { //deteccion ausencia del dedo 
        irBaseline += 0.05f * ((float)ir - irBaseline);
        redBaseline += 0.05f * ((float)red - redBaseline);
        signalFiltered = 0.0f;
        resetPulseState();
        return;
    }

    //estimación de Spo2 
    irBaseline += 0.01f * ((float)ir - irBaseline);
    redBaseline += 0.01f * ((float)red - redBaseline);

    float irAc = (float)ir - irBaseline;
    float redAc = (float)red - redBaseline;
//suaviza la señal 
    if (irAc > irAcMax) irAcMax = irAc;
    if (irAc < irAcMin) irAcMin = irAc;
    if (redAc > redAcMax) redAcMax = redAc;
    if (redAc < redAcMin) redAcMin = redAc;

    acWindowSamples++;
    if (acWindowSamples >= AC_WINDOW_SIZE) {
        updateSpO2Estimate(irAcMax - irAcMin, redAcMax - redAcMin);
        acWindowSamples = 0;
        irAcMax = -1e9f;
        irAcMin = 1e9f;
        redAcMax = -1e9f;
        redAcMin = 1e9f;
    }

    signalFiltered += 0.25f * (irAc - signalFiltered);

    amplitude += 0.02f * (fabsf(signalFiltered) - amplitude);
    threshold = amplitude * 0.45f;
    if (threshold < 20.0f) {
        threshold = 20.0f;
    }

    sample0 = sample1;
    sample1 = sample2;
    sample2 = signalFiltered;

    if (sampleCounter < 200) { //espera al menos 200 muestras para estabilizar los filtros 
        return;
    }

    bool peakDetected = (sample1 > sample0) &&
                        (sample1 >= sample2) &&
                        (sample1 > threshold);
    if (!peakDetected) {
        return;
    }

    unsigned long peakSample = sampleCounter - 1;
    if (lastBeatSample == 0) {
        lastBeatSample = peakSample;
        return;
    }

    unsigned long sampleInterval = peakSample - lastBeatSample;
    float intervalMs = (sampleInterval * 1000.0f) / SAMPLE_RATE_HZ;
// Calculo de intervalo en milisegundos
    if (intervalMs < 300.0f) {
        return;
    }
    if (intervalMs > 1500.0f) {
        lastBeatSample = peakSample;
        return;
    }

    float newBPM = 60000.0f / intervalMs;
    //valida el rango y promedia los ultimos latidos 
    if (newBPM >= 40.0f && newBPM <= 200.0f) {
        bpm = newBPM;
        rates[rateSpot] = bpm;
        rateSpot = (rateSpot + 1) % RATE_SIZE;
        if (validBeats < RATE_SIZE) {
            validBeats++;
        }

        float sum = 0.0f;
        for (byte i = 0; i < validBeats; i++) {
            sum += rates[i];
        }
        bpmAverage = sum / validBeats;
        lastBeatSample = peakSample; 
    }
}

void updateSw420(unsigned long now) { //detectar vibraciones 
    if (digitalRead(SW420_PIN) == HIGH) {
        lastVibrationMs = now; 
        vibrationActive = true;
    } else if (vibrationActive && (now - lastVibrationMs >= VIBRATION_HOLD_MS)) {
        vibrationActive = false;
    }
}

void evaluateSingleAlert(unsigned long now) { //evalua la condicion de la alarma 
    if (buzzerManualTest && now < manualBuzzerUntil) { // activa la alarma en modo prueba
        alertActive = true;
        alertReason = "PRUEBA_ACTUADOR";
        return;
    }
    buzzerManualTest = false;

    bool fingerPresent = (currentIR >= 15000); 
    alertActive = false;
    alertReason = "NONE";

    if (!buzzerMuted && fingerPresent && spo2Est > 0.0f && spo2Est < spo2Threshold) { // establece el rango de activacion 
        alertActive = true;
        alertReason = "SPO2_BAJO";
    }
}

void updateBuzzer(unsigned long now) { // actualiza el estado del buzzer de forma no bloqueante 
    evaluateSingleAlert(now); 
    if (!alertActive) {
        buzzerState = false;
        digitalWrite(BUZZER_PIN, LOW);
        return;
    }

    if (now - lastBuzzerToggle >= 300) {
        lastBuzzerToggle = now;
        buzzerState = !buzzerState;
        digitalWrite(BUZZER_PIN, buzzerState ? HIGH : LOW);
    }
}

void updateOled(unsigned long now) { //actualizacion de la informacion pantalla OLED 
    if (!oledOk || (now - lastOledUpdate < OLED_INTERVAL)) return; //validacion conexion exitosa pantalla 
    lastOledUpdate = now;

    bool fingerPresent = (currentIR >= 15000);

    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.printf("SD:%s SW:%s",
                !sdOk ? "NO" : (sdLoggingEnabled ? "OK" : "PAU"),
                vibrationActive ? "MOV" : "OK"); //estado superior 

    oled.setCursor(0, 14);
    if (!fingerPresent) {
        oled.println("Coloque el dedo...");
        oled.setCursor(0, 30);
        oled.println("FC: -- BPM");
        oled.setCursor(0, 46);
        oled.printf("SpO2: -- (Min:%.0f)", spo2Threshold);
    } else { //muestra la infromacion de FC y Spo2
        oled.setTextSize(2); 
        oled.setCursor(0, 22);
        if (bpmAverage > 0.0f) oled.printf("FC:%.0f", bpmAverage);
        else oled.print("FC:--");

        oled.setCursor(0, 44);
        if (spo2Est > 0.0f) oled.printf("O2:%.0f%%", spo2Est);
        else oled.print("O2:--%");
    }

    oled.display();
}

void updateSdLog(unsigned long now) { //registro periodico en SD
    if (!sdOk || !sdLoggingEnabled || (now - lastSdWrite < SD_INTERVAL)) return; //validacion estado SD 
    lastSdWrite = now; 

    String row = String(now) + "," + //marca de tiempo
                 String(sampleCounter) + "," + // contador de muestras
                 String(currentIR) + "," + //señal infrarojo
                 String(currentRED) + "," + //señal rojo
                 String(signalFiltered, 2) + "," + // señal filtrada
                 String(bpmAverage, 1) + "," + // promedio fc 
                 String(spo2Est, 1) + "," + // saturacion de oxigeno
                 String(spo2Threshold, 1) + "," + // umbral oxigeno
                 String(vibrationActive ? 1 : 0) + "," + // estado de vibracion
                 String(alertReason); //motivo de la alrma 

    appendToSD("/registro.csv", row);
}

void updateSerial(unsigned long now) { // imprime periódicamente en el puerto serie
    if (now - lastSerialPrint < SERIAL_INTERVAL) return;
    lastSerialPrint = now;

    Serial.printf("FC:%.1f | SpO2:%.1f%% (Min:%.1f) | SW420:%d | Alerta:%s | SD:%s\n",
                  bpmAverage, spo2Est, spo2Threshold, vibrationActive ? 1 : 0,
                  alertReason, !sdOk ? "NO" : (sdLoggingEnabled ? "ON" : "PAUSA"));
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== INICIO MONITOR INALAMBRICO BIDIRECCIONAL ===");

    pinMode(SW420_PIN, INPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // 1. Iniciar Access Point local
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("AP activo: ");
    Serial.print(AP_SSID);
    Serial.print(" | IP: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.on("/data", handleData);
    server.on("/config", handleConfig);
    server.begin();

    // 2. Iniciar OLED en segundo bus I2C (25 y 26)
    BusOLED.begin(OLED_SDA, OLED_SCL);
    BusOLED.setClock(400000);
    if (oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        oledOk = true;
        oled.clearDisplay();
        oled.setTextSize(1);
        oled.setTextColor(SSD1306_WHITE);
        oled.setCursor(0, 0);
        oled.println("AP: ESP32_Pulse_Sensor");
        oled.setCursor(0, 12);
        oled.println("IP: 192.168.4.1");
        oled.display();
    } else {
        oledOk = false;
        Serial.println("OLED no detectada.");
    }

    // 3. Iniciar MAX30102 en bus principal (21 y 22)
    Wire.begin(SENSOR_SDA, SENSOR_SCL);
    if (!sensor.begin(Wire, I2C_SPEED_STANDARD)) {
        Serial.println("ERROR: MAX30102 no detectado.");
        while (true) { delay(1000); }
    }

    sensor.setup(0x3F, 1, 2, 100, 411, 4096);
    sensor.setPulseAmplitudeRed(0x3F);
    sensor.setPulseAmplitudeIR(0x3F);
    sensor.setPulseAmplitudeGreen(0);

    // 4. Iniciar MicroSD (SPI)
    if (SD.begin(SD_CS)) {
        sdOk = true;
        Serial.println("MicroSD lista.");
        if (!SD.exists("/registro.csv")) {
            appendToSD("/registro.csv", "Tiempo_ms,Muestra,IR,RED,SignalFiltered,BPM_Avg,SpO2_Pct,Umbral_SpO2,Vibracion_SW420,Alerta");
        }
    } else {
        sdOk = false;
        Serial.println("MicroSD no detectada.");
    }
}

void loop() {
    unsigned long now = millis();
    server.handleClient();

    sensor.check();
    while (sensor.available()) {
        long ir = sensor.getFIFOIR();
        long red = sensor.getFIFORed();
        sensor.nextSample();
        processSample(ir, red);
    }

    updateSw420(now);
    updateBuzzer(now);
    updateOled(now);
    updateSdLog(now);
    updateSerial(now);
}