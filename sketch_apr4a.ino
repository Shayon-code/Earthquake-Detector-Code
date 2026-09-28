#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

const char* ssid     = "OM ENERGY C312";
const char* password = "px4tdkpx4tdk";

WebServer server(80);

// ================================================================
//  PINS
// ================================================================
#define BUZZER_PIN   4
#define LED_PIN      2
#define X_PIN        34
#define Y_PIN        35
#define Z_PIN        32
#define BATTERY_PIN  13

// ================================================================
//  ADXL335B — 1g = 372.7 ADC counts at 3.3V
// ================================================================
#define SCALE_G  (1.0f / 372.7f)

// ================================================================
//  DETECTION SETTINGS
// ================================================================
#define P_STA_WINDOW          10
#define P_LTA_WINDOW          3000
#define P_TRIGGER_MULTIPLIER  2.0f
#define P_TRIGGER_MIN         1.5f
#define P_TRIGGER_MAX         4.0f
#define P_CONFIRM_SAMPLES     20    // 20 x 5ms = 100ms sustained

#define S_STA_WINDOW          20
#define S_LTA_WINDOW          6000
#define S_TRIGGER_MULTIPLIER  3.5f
#define S_TRIGGER_MIN         2.0f
#define S_TRIGGER_MAX         8.0f
#define S_CONFIRM_SAMPLES     30    // 30 x 5ms = 150ms sustained

#define NOISE_GATE_G          0.003f
#define LPF_ALPHA             0.12f
#define SAMPLE_INTERVAL_MS    5
#define CALIBRATION_SAMPLES   500
#define OBSERVE_SAMPLES       500
#define WARNING_DURATION_MS   15000UL
#define ALERT_DURATION_MS     8000UL
#define COOLDOWN_MS           10000UL

// ================================================================
//  GLOBALS
// ================================================================
int baseX = 0, baseY = 0, baseZ = 0;
int deltaX = 0, deltaY = 0, deltaZ = 0;

float lpf_x = 0, lpf_y = 0, lpf_z = 0;
float h_mag  = 0;
float td_mag = 0;

float p_sta_buf[P_STA_WINDOW];
float p_lta_buf[P_LTA_WINDOW];
int   p_sta_idx = 0, p_lta_idx = 0;
float p_sta_sum = 0, p_lta_sum = 0;
float p_trigger = 2.0f;
float p_ratio   = 0;
int   p_confirm = 0;

float s_sta_buf[S_STA_WINDOW];
float s_lta_buf[S_LTA_WINDOW];
int   s_sta_idx = 0, s_lta_idx = 0;
float s_sta_sum = 0, s_lta_sum = 0;
float s_trigger = 3.0f;
float s_ratio   = 0;
int   s_confirm = 0;

bool pWaveDetected      = false;
bool earthquakeDetected = false;
bool in_cooldown        = false;
bool warning_active     = false;
bool alert_active       = false;
bool buzzerOn           = false;

unsigned long event_start_ms   = 0;
unsigned long event_end_ms     = 0;
unsigned long p_detect_ms      = 0;
unsigned long warning_start_ms = 0;
unsigned long alert_start_ms   = 0;
unsigned long last_sample_ms   = 0;
unsigned long last_batt_ms     = 0;
unsigned long last_lcd_ms      = 0;
unsigned long last_serial_ms   = 0;
unsigned long last_ip_ms       = 0;

float batteryV = 0;

// ================================================================
//  STA/LTA
// ================================================================
float sta_lta_update(float sample,
  float* sta_buf, int& sta_idx, float& sta_sum, int sta_win,
  float* lta_buf, int& lta_idx, float& lta_sum, int lta_win) {

  float e = sample * sample;
  sta_sum -= sta_buf[sta_idx];
  sta_buf[sta_idx] = e;
  sta_sum += e;
  sta_idx = (sta_idx + 1) % sta_win;

  lta_sum -= lta_buf[lta_idx];
  lta_buf[lta_idx] = e;
  lta_sum += e;
  lta_idx = (lta_idx + 1) % lta_win;

  float lta = lta_sum / lta_win;
  float sta = sta_sum / sta_win;
  return (lta < 1e-6f) ? 0.0f : sta / lta;
}

// ================================================================
//  CALIBRATION
// ================================================================
void calibrateSensor() {

  // Phase 1 — baseline
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calibrating 1/3");
  lcd.setCursor(0, 1); lcd.print("Keep still!     ");
  Serial.println("\n[CAL] Phase 1 — reading baseline (keep still)...");

  long sx = 0, sy = 0, sz = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    sx += analogRead(X_PIN);
    sy += analogRead(Y_PIN);
    sz += analogRead(Z_PIN);
    delay(5);
  }
  baseX = sx / CALIBRATION_SAMPLES;
  baseY = sy / CALIBRATION_SAMPLES;
  baseZ = sz / CALIBRATION_SAMPLES;
  Serial.printf("[CAL] Resting ADC — X:%d  Y:%d  Z:%d\n", baseX, baseY, baseZ);
  Serial.printf("[CAL] Resting g   — X:%.4f  Y:%.4f  Z:%.4f\n",
    (baseX-2048)*SCALE_G, (baseY-2048)*SCALE_G, (baseZ-2048)*SCALE_G);

  // Phase 2 — fill buffers
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calibrating 2/3");
  lcd.setCursor(0, 1); lcd.print("                ");
  Serial.println("[CAL] Phase 2 — filling detection buffers...");

  float h_acc = 0, td_acc = 0;
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float gx = (analogRead(X_PIN) - baseX) * SCALE_G;
    float gy = (analogRead(Y_PIN) - baseY) * SCALE_G;
    float gz = (analogRead(Z_PIN) - baseZ) * SCALE_G;
    lpf_x = LPF_ALPHA * gx + (1 - LPF_ALPHA) * lpf_x;
    lpf_y = LPF_ALPHA * gy + (1 - LPF_ALPHA) * lpf_y;
    lpf_z = LPF_ALPHA * gz + (1 - LPF_ALPHA) * lpf_z;
    float hm  = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y);
    float tdm = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y + lpf_z*lpf_z);
    h_acc  += hm  * hm;
    td_acc += tdm * tdm;
    if (i % 50 == 0) { lcd.setCursor(i/50, 1); lcd.print("-"); }
    delay(5);
  }

  float h_mean  = h_acc  / CALIBRATION_SAMPLES;
  float td_mean = td_acc / CALIBRATION_SAMPLES;

  for (int i = 0; i < S_STA_WINDOW; i++) s_sta_buf[i] = h_mean;
  for (int i = 0; i < S_LTA_WINDOW; i++) s_lta_buf[i] = h_mean;
  s_sta_sum = h_mean * S_STA_WINDOW;
  s_lta_sum = h_mean * S_LTA_WINDOW;

  for (int i = 0; i < P_STA_WINDOW; i++) p_sta_buf[i] = td_mean;
  for (int i = 0; i < P_LTA_WINDOW; i++) p_lta_buf[i] = td_mean;
  p_sta_sum = td_mean * P_STA_WINDOW;
  p_lta_sum = td_mean * P_LTA_WINDOW;
  Serial.println("[CAL] Buffers filled — ratios start at 1.000");

  // Phase 3 — observe quiet ratio
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Calibrating 3/3");
  lcd.setCursor(0, 1); lcd.print("                ");
  Serial.println("[CAL] Phase 3 — measuring ambient noise (keep still)...");

  float max_p = 0, max_s = 0;
  for (int i = 0; i < OBSERVE_SAMPLES; i++) {
    float gx = (analogRead(X_PIN) - baseX) * SCALE_G;
    float gy = (analogRead(Y_PIN) - baseY) * SCALE_G;
    float gz = (analogRead(Z_PIN) - baseZ) * SCALE_G;
    lpf_x = LPF_ALPHA * gx + (1 - LPF_ALPHA) * lpf_x;
    lpf_y = LPF_ALPHA * gy + (1 - LPF_ALPHA) * lpf_y;
    lpf_z = LPF_ALPHA * gz + (1 - LPF_ALPHA) * lpf_z;
    float hm  = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y);
    float tdm = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y + lpf_z*lpf_z);
    float pr = sta_lta_update(tdm, p_sta_buf, p_sta_idx, p_sta_sum, P_STA_WINDOW, p_lta_buf, p_lta_idx, p_lta_sum, P_LTA_WINDOW);
    float sr = sta_lta_update(hm,  s_sta_buf, s_sta_idx, s_sta_sum, S_STA_WINDOW, s_lta_buf, s_lta_idx, s_lta_sum, S_LTA_WINDOW);
    if (pr > max_p) max_p = pr;
    if (sr > max_s) max_s = sr;
    if (i % 50 == 0) { lcd.setCursor(i/50, 1); lcd.print("*"); }
    delay(5);
  }

  p_trigger = constrain(max_p * P_TRIGGER_MULTIPLIER, P_TRIGGER_MIN, P_TRIGGER_MAX);
  s_trigger = constrain(max_s * S_TRIGGER_MULTIPLIER, S_TRIGGER_MIN, S_TRIGGER_MAX);

  Serial.println("[CAL] ==========================================");
  Serial.printf("[CAL]  Max ambient P ratio  : %.4f\n", max_p);
  Serial.printf("[CAL]  P trigger threshold  : %.4f  (ambient x %.1f)\n", p_trigger, P_TRIGGER_MULTIPLIER);
  Serial.printf("[CAL]  Max ambient S ratio  : %.4f\n", max_s);
  Serial.printf("[CAL]  S trigger threshold  : %.4f  (ambient x %.1f)\n", s_trigger, S_TRIGGER_MULTIPLIER);
  Serial.printf("[CAL]  P needs %d samples above trigger = %dms\n", P_CONFIRM_SAMPLES, P_CONFIRM_SAMPLES * SAMPLE_INTERVAL_MS);
  Serial.printf("[CAL]  S needs %d samples above trigger = %dms\n", S_CONFIRM_SAMPLES, S_CONFIRM_SAMPLES * SAMPLE_INTERVAL_MS);
  Serial.println("[CAL] ==========================================\n");

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("P:"); lcd.print(p_trigger, 2);
  lcd.print(" S:"); lcd.print(s_trigger, 2);
  lcd.setCursor(0, 1); lcd.print("Ready!");
  delay(2000);
}

// ================================================================
//  ALERTS
// ================================================================
void triggerWarning() {
  warning_active   = true;
  warning_start_ms = millis();
  tone(BUZZER_PIN, 500);
  digitalWrite(LED_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("!! P-WAVE !!    ");
  lcd.setCursor(0, 1); lcd.print("EVACUATE NOW!   ");
  Serial.println("--------------------------------------------");
  Serial.printf("[P-WAVE DETECTED] Ratio:%.3f | Magnitude:%.5fg\n", p_ratio, td_mag);
  Serial.println("--------------------------------------------");
}

void triggerEarthquake() {
  alert_active   = true;
  alert_start_ms = millis();
  buzzerOn       = true;
  tone(BUZZER_PIN, 1000);
  digitalWrite(LED_PIN, HIGH);
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("!! EARTHQUAKE !!");
  lcd.setCursor(0, 1); lcd.print("TAKE COVER NOW! ");
  Serial.println("============================================");
  Serial.printf("[EARTHQUAKE CONFIRMED] S-ratio:%.3f | H-Magnitude:%.5fg\n", s_ratio, h_mag);
  if (pWaveDetected) {
    Serial.printf("[S-P INTERVAL] %lums of warning time\n", millis() - p_detect_ms);
  }
  Serial.println("============================================");
}

void check_alert_timeout() {
  unsigned long now = millis();

  if (warning_active && !alert_active) {
    if (now - warning_start_ms >= WARNING_DURATION_MS) {
      warning_active = false;
      pWaveDetected  = false;
      noTone(BUZZER_PIN);
      digitalWrite(LED_PIN, LOW);
      lcd.clear(); lcd.print("X    Y    Z");
      Serial.println("[P-WAVE] Warning expired — no earthquake followed");
    }
  }

  if (alert_active) {
    unsigned long elapsed = now - alert_start_ms;
    if (elapsed >= ALERT_DURATION_MS && buzzerOn) {
      noTone(BUZZER_PIN);
      buzzerOn = false;
    }
    if (elapsed >= ALERT_DURATION_MS + 3000) {
      alert_active   = false;
      warning_active = false;
      digitalWrite(LED_PIN, LOW);
      lcd.clear(); lcd.print("X    Y    Z");
    }
  }
}

// ================================================================
//  SENSOR + DETECTION
// ================================================================
void readSensor() {
  unsigned long now = millis();
  if (now - last_sample_ms < SAMPLE_INTERVAL_MS) return;
  last_sample_ms = now;

  int dx = analogRead(X_PIN) - baseX;
  int dy = analogRead(Y_PIN) - baseY;
  int dz = analogRead(Z_PIN) - baseZ;

  deltaX = dx; deltaY = dy; deltaZ = dz;

  lpf_x = LPF_ALPHA * (dx * SCALE_G) + (1 - LPF_ALPHA) * lpf_x;
  lpf_y = LPF_ALPHA * (dy * SCALE_G) + (1 - LPF_ALPHA) * lpf_y;
  lpf_z = LPF_ALPHA * (dz * SCALE_G) + (1 - LPF_ALPHA) * lpf_z;

  h_mag  = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y);
  td_mag = sqrtf(lpf_x*lpf_x + lpf_y*lpf_y + lpf_z*lpf_z);

  p_ratio = sta_lta_update(td_mag, p_sta_buf, p_sta_idx, p_sta_sum, P_STA_WINDOW, p_lta_buf, p_lta_idx, p_lta_sum, P_LTA_WINDOW);
  s_ratio = sta_lta_update(h_mag,  s_sta_buf, s_sta_idx, s_sta_sum, S_STA_WINDOW, s_lta_buf, s_lta_idx, s_lta_sum, S_LTA_WINDOW);

  // Cooldown check
  if (in_cooldown && now - event_end_ms >= COOLDOWN_MS) {
    in_cooldown = false;
    Serial.println("[SYSTEM] Cooldown finished — monitoring resumed");
  }

  // P-wave detection
  if (!pWaveDetected && !in_cooldown) {
    if (p_ratio >= p_trigger && td_mag >= NOISE_GATE_G) {
      p_confirm++;
      // p_confirm counts how many consecutive samples are above the trigger
      // when it reaches P_CONFIRM_SAMPLES the P-wave is confirmed
    } else {
      p_confirm = 0;
    }
    if (p_confirm >= P_CONFIRM_SAMPLES) {
      pWaveDetected = true;
      p_detect_ms   = now;
      p_confirm     = 0;
      triggerWarning();
    }
  }

  // S-wave detection
  if (!earthquakeDetected && !in_cooldown) {
    if (s_ratio >= s_trigger && h_mag >= NOISE_GATE_G) {
      s_confirm++;
    } else {
      s_confirm = 0;
    }
    if (s_confirm >= S_CONFIRM_SAMPLES) {
      earthquakeDetected = true;
      event_start_ms     = now;
      s_confirm          = 0;
      triggerEarthquake();
    }
  }

  // Event end
  if (earthquakeDetected && s_ratio < 1.0f) {
    earthquakeDetected = false;
    pWaveDetected      = false;
    event_end_ms       = now;
    in_cooldown        = true;
    Serial.printf("[EVENT ENDED] Duration: %lums — cooldown started\n",
      now - event_start_ms);
  }

  // ----------------------------------------------------------------
  //  SERIAL MONITOR — every 100ms
  //
  //  Format:
  //  STATUS | 3D magnitude | H magnitude | P-ratio/trigger (count/needed) | S-ratio/trigger (count/needed)
  //
  //  STATUS:
  //    SAFE        = nothing detected
  //    P-WAVE(xx%) = building up to P-wave, xx% of confirmation
  //    P-WAVE OK   = P-wave confirmed, waiting for S-wave
  //    S-WAVE(xx%) = building up to earthquake, xx% of confirmation
  //    EARTHQUAKE  = confirmed earthquake
  //    cooldown    = recovering after event
  // ----------------------------------------------------------------
  if (now - last_serial_ms >= 100) {
    last_serial_ms = now;

    // Work out status label
    String status;
    if (earthquakeDetected) {
      status = "EARTHQUAKE  ";
    } else if (pWaveDetected) {
      if (s_confirm > 0) {
        int pct = (s_confirm * 100) / S_CONFIRM_SAMPLES;
        status = "S-WAVE(" + String(pct) + "%)  ";
      } else {
        status = "P-WAVE OK   ";
      }
    } else if (in_cooldown) {
      status = "cooldown    ";
    } else if (p_confirm > 0) {
      int pct = (p_confirm * 100) / P_CONFIRM_SAMPLES;
      status = "P-WAVE(" + String(pct) + "%)  ";
    } else {
      status = "SAFE        ";
    }

    Serial.printf("[%s] 3D:%.4fg H:%.4fg | P:%.2f/%.2f | S:%.2f/%.2f\n",
      status.c_str(),
      td_mag, h_mag,
      p_ratio, p_trigger,
      s_ratio, s_trigger);
  }

  // LCD every 500ms
  if (!alert_active && !warning_active && now - last_lcd_ms >= 500) {
    last_lcd_ms = now;
    lcd.setCursor(0, 0); lcd.print("X    Y    Z     ");
    lcd.setCursor(0, 1);
    String sx = String(abs(deltaX)); while(sx.length()<4) sx=" "+sx;
    String sy = String(abs(deltaY)); while(sy.length()<4) sy=" "+sy;
    String sz = String(abs(deltaZ)); while(sz.length()<4) sz=" "+sz;
    lcd.print(sx); lcd.setCursor(5,1);
    lcd.print(sy); lcd.setCursor(10,1);
    lcd.print(sz);
  }
}

// ================================================================
//  BATTERY
// ================================================================
void checkBattery() {
  if (millis() - last_batt_ms < 5000) return;
  last_batt_ms = millis();
  batteryV = (analogRead(BATTERY_PIN) / 4095.0f) * 3.3f * 2.0f;
  Serial.printf("[BATTERY] %.2fV\n", batteryV);
  if (batteryV < 3.3f && !alert_active && !warning_active) {
    lcd.clear(); lcd.print("Battery Low!");
    tone(BUZZER_PIN, 2000, 500);
    delay(1500);
    lcd.clear(); lcd.print("X    Y    Z");
  }
}

// ================================================================
//  WEB PAGE
// ================================================================
void handleRoot() {
  server.send(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="2">
  <title>Earthquake Detector</title>
  <style>
    body { font-family: Arial; text-align: center; background: #111; color: #eee; padding: 30px; }
    h1   { font-size: 1.4em; margin-bottom: 20px; }
    #status { font-size: 2em; font-weight: bold; padding: 20px; border-radius: 12px; margin-bottom: 20px; }
    .safe     { background: #1a4a1a; color: #00ee55; }
    .warning  { background: #4a4a00; color: #ffdd00; }
    .danger   { background: #4a0000; color: #ff3333; }
    .cooldown { background: #001a4a; color: #4488ff; }
    .readings { font-size: 1.1em; margin: 10px 0; }
    .ratios   { font-size: 0.9em; color: #aaa; margin: 10px 0; }
    button { padding: 12px 30px; font-size: 1em; background: #cc2200; color: white; border: none; border-radius: 8px; cursor: pointer; margin-top: 20px; }
  </style>
</head>
<body>
  <h1>Earthquake Detector</h1>
  <div id="status" class="safe">ALL CLEAR</div>
  <div class="readings">
    X: <b id="x">0</b> &nbsp; Y: <b id="y">0</b> &nbsp; Z: <b id="z">0</b>
  </div>
  <div class="ratios">
    P-wave ratio: <span id="pr">0.00</span> / <span id="pt">--</span>
    &nbsp;|&nbsp;
    S-wave ratio: <span id="sr">0.00</span> / <span id="st">--</span>
  </div>
  <div class="ratios">Battery: <span id="batt">--</span>V</div>
  <button onclick="fetch('/reset')">Reset Alarm</button>
<script>
function update() {
  fetch('/data').then(r => r.json()).then(d => {
    document.getElementById('x').textContent    = d.deltaX;
    document.getElementById('y').textContent    = d.deltaY;
    document.getElementById('z').textContent    = d.deltaZ;
    document.getElementById('pr').textContent   = d.pRatio.toFixed(2);
    document.getElementById('pt').textContent   = d.pTrigger.toFixed(2);
    document.getElementById('sr').textContent   = d.sRatio.toFixed(2);
    document.getElementById('st').textContent   = d.sTrigger.toFixed(2);
    document.getElementById('batt').textContent = d.battery.toFixed(2);
    let s = document.getElementById('status');
    if      (d.earthquake) { s.textContent = 'EARTHQUAKE!';     s.className = 'danger'; }
    else if (d.pWave)      { s.textContent = 'P-WAVE WARNING!'; s.className = 'warning'; }
    else if (d.cooldown)   { s.textContent = 'COOLDOWN...';     s.className = 'cooldown'; }
    else                   { s.textContent = 'ALL CLEAR';       s.className = 'safe'; }
  });
}
setInterval(update, 2000);
update();
</script>
</body>
</html>)rawliteral");
}

void handleData() {
  String j = "{";
  j += "\"deltaX\":"     + String(deltaX)      + ",";
  j += "\"deltaY\":"     + String(deltaY)      + ",";
  j += "\"deltaZ\":"     + String(deltaZ)      + ",";
  j += "\"pRatio\":"     + String(p_ratio,   2) + ",";
  j += "\"sRatio\":"     + String(s_ratio,   2) + ",";
  j += "\"pTrigger\":"   + String(p_trigger, 2) + ",";
  j += "\"sTrigger\":"   + String(s_trigger, 2) + ",";
  j += "\"battery\":"    + String(batteryV,  2) + ",";
  j += "\"pWave\":"      + String(pWaveDetected      ? "true":"false") + ",";
  j += "\"cooldown\":"   + String(in_cooldown         ? "true":"false") + ",";
  j += "\"earthquake\":" + String(earthquakeDetected  ? "true":"false");
  j += "}";
  server.send(200, "application/json", j);
}

void handleReset() {
  earthquakeDetected = pWaveDetected = alert_active = warning_active = in_cooldown = false;
  p_confirm = s_confirm = 0;
  buzzerOn  = false;
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, LOW);
  lcd.clear(); lcd.print("X    Y    Z");
  server.send(200, "text/plain", "Reset OK");
  Serial.println("[SYSTEM] Manual reset");
}

// ================================================================
//  SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0); lcd.print("Booting...");

  pinMode(BUZZER_PIN,  OUTPUT);
  pinMode(LED_PIN,     OUTPUT);
  pinMode(BATTERY_PIN, INPUT);

  memset(p_sta_buf, 0, sizeof(p_sta_buf));
  memset(p_lta_buf, 0, sizeof(p_lta_buf));
  memset(s_sta_buf, 0, sizeof(s_sta_buf));
  memset(s_lta_buf, 0, sizeof(s_lta_buf));

  calibrateSensor();

  WiFi.begin(ssid, password);
  lcd.clear(); lcd.print("WiFi...");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500); Serial.print("."); tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n=============================");
    Serial.print("  Dashboard: http://");
    Serial.println(WiFi.localIP());
    Serial.println("=============================");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("WiFi OK!");
    lcd.setCursor(0, 1); lcd.print(WiFi.localIP());
    delay(3000);
  } else {
    Serial.println("\n[WiFi] Failed — running offline");
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("No WiFi - OK");
    lcd.setCursor(0, 1); lcd.print("Running offline");
    delay(2000);
  }

  server.on("/",      handleRoot);
  server.on("/data",  handleData);
  server.on("/reset", handleReset);
  server.begin();

  lcd.clear(); lcd.print("X    Y    Z");
  Serial.println("[SYSTEM] Ready — monitoring started");
  Serial.printf("[SYSTEM] P trigger:%.2f | S trigger:%.2f\n", p_trigger, s_trigger);
  Serial.println("[SYSTEM] Serial format: [STATUS] 3D | H | P-ratio/trigger | S-ratio/trigger\n");
}

// ================================================================
//  LOOP
// ================================================================
void loop() {
  server.handleClient();
  readSensor();
  checkBattery();
  check_alert_timeout();

  if (millis() - last_ip_ms >= 30000 && WiFi.status() == WL_CONNECTED) {
    last_ip_ms = millis();
    Serial.print("[IP] http://"); Serial.println(WiFi.localIP());
  }
}