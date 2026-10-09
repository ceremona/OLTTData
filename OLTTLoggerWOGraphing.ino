/* ============================================================================
 *  OLTTLogger_Calibrated.ino
 *  ---------------------------------------------------------------------------
 *  Temperature + pressure logger for the OLTT rig (All Power Labs).
 *  Derived from OLTTLoggerWOGraphing.ino  (Cere Davis, 12 July 2019).
 *
 *  ---------------------------------------------------------------------------
 *  TARGET HARDWARE
 *    MCU ....... Teensy 3.x, 3.3 V logic. No level shifter needed.
 *                  pin 18 = A4 / SDA0   -> TCA9548A SDA (pin 23)
 *                  pin 19 = A5 / SCL0   -> TCA9548A SCL (pin 22)
 *                  pin 22 = A8          -> 1-Wire DQ (all MAX31850K, pin 5)
 *                  pin 13 = LED_BUILTIN -> status blink
 *    Temp ...... 5x MAX31850K K-type thermocouple amps, family code 0x3B,
 *                on ONE shared 1-Wire bus. Library: Adafruit OneWire +
 *                DallasTemperature (MAX31850 forks). Fixed 14-bit / 0.25 C.
 *                DQ is 3 V-only; T- (pin 2) is internally biased, NEVER ground.
 *                Power each breakout on VDD (pin 4) -- NOT parasite power.
 *                Exactly ONE 4.7 k pull-up for the whole bus.
 *    Pressure .. 3x Honeywell MPRLS (0x18) behind a DFRobot TCA9548A mux
 *                (0x70): CH1 -> P_mixing_chamber, CH2 -> P_rotometer,
 *                CH5 -> P_separator_column. MPRLS: pin 2 = SDA, pin 3 = SCL,
 *                pin 10 = VSS, pin 12 = VDD. EOC(8)/RES(9) unused.
 *
 *  ---------------------------------------------------------------------------
 *  WHAT THIS VERSION CHANGES
 *   (1) SPIKES REMOVED, NOT SMOOTHED. No EMA, no moving average anywhere.
 *       A median-of-N returns one of the ACTUAL samples, so the value you log
 *       is a real measurement with zero blending. Isolated spikes vanish.
 *   (2) CALIBRATION KEYED BY ROM ADDRESS, not bus index. On a shared bus the
 *       enumeration order is not guaranteed stable; if one amp drops and
 *       re-enumerates, index-based calibration silently lands on the wrong
 *       channel. Fixed slots fix that.
 *   (3) ICE-BATH + BOILING-BATH calibration built in, driven over the USB
 *       serial console, stored in EEPROM. The hot point uses the LOCAL boiling
 *       point of water (from ambient absolute pressure), not a hard 100 C.
 *   (4) TRUE 5 s CADENCE from millis(), not blocking delays.
 *   (5) Faults (-127 / -254 / -251 / -250) are DROPPED and counted, not
 *       filtered -- an open junction is a wiring event, not noise.
 * ========================================================================== */

#include "DFRobot_I2CMultiplexer.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_MPRLS.h"

/* ---- pin map (unchanged from the original sketch) ----------------------- */
#define ONE_WIRE_BUS     22      // Teensy pin 22 -> 1-Wire DQ
#define PRESSURE_SCA     18      // retained for reference (I2C0 SDA)
#define PRESSURE_SCL     19      // retained for reference (I2C0 SCL)

/* ---- tunables ----------------------------------------------------------- */
#define N_TEMP           5              // thermocouple channels
#define N_PRESS          3              // pressure channels
#define LOG_INTERVAL_MS  5000UL         // log cadence: 5 s
#define MEDIAN_N         3              // odd. 3 = fast, 5 = stronger. see notes.
#define CAL_CAPTURE_MS   6000UL         // averaging window per calibration point

/* Spike gate: EXTRA rejection of a sustained jump (stuck / shorted input).
 * NAN = median-only (default). Set e.g. 50.0f to enable. This is not smoothing;
 * it rejects a sample that is implausibly far from the current median. */
#define SPIKE_GATE_C     NAN
#define SPIKE_GATE_KPA   NAN

/* ---- DallasTemperature fault codes -------------------------------------- */
/* All are <= -127, so "raw <= DEVICE_DISCONNECTED_C" catches every one. */
#ifndef DEVICE_DISCONNECTED_C
  #define DEVICE_DISCONNECTED_C       -127   // device not responding
#endif
#ifndef DEVICE_FAULT_OPEN_C
  #define DEVICE_FAULT_OPEN_C         -254   // open thermocouple / broken lead
#endif
#ifndef DEVICE_POWER_ON_RESET_C
  #define DEVICE_POWER_ON_RESET_C     -251
#endif
#ifndef DEVICE_INSUFFICIENT_POWER_C
  #define DEVICE_INSUFFICIENT_POWER_C -250   // usually parasite-power droop
#endif

DFRobot_I2CMultiplexer I2CMulti(0x70);
Adafruit_MPRLS        mpr = Adafruit_MPRLS(-1, -1);   // no RESET/EOC pins
OneWire               oneWireBus(ONE_WIRE_BUS);
DallasTemperature     oneWireSensors(&oneWireBus);

/* ==========================================================================
 *  SpikeFilter<N> -- median-of-N, optional gate.
 *
 *  WHY A MEDIAN AND NOT AN EMA:
 *    An EMI spike on a long thermocouple lead, or a bus glitch, is an OUTLIER.
 *    An average is dragged toward it and needs several samples to recover; a
 *    median ignores it, because the middle of the window is a real sample.
 *    We therefore REMOVE the spike; we do not blend it into the trace.
 * ========================================================================== */
template <uint8_t N>
class SpikeFilter {
public:
  SpikeFilter() : _gate(NAN) {}
  void    setGate(float g) { _gate = g; }
  float   value() const    { return _last; }
  uint8_t count() const    { return _n; }

  float update(float x) {
    if (!isnan(_gate) && _n >= 3) {                 // gate: need a baseline first
      float med = median();
      if (fabsf(x - med) > _gate) { _last = med; return _last; }   // reject, hold
    }
    _buf[_idx] = x;
    _idx = (_idx + 1) % N;
    if (_n < N) _n++;
    _last = median();
    return _last;
  }

private:
  float median() const {
    float t[N];
    for (uint8_t i = 0; i < _n; i++) t[i] = _buf[i];
    for (uint8_t i = 1; i < _n; i++) {              // insertion sort; N is tiny
      float key = t[i]; int8_t j = i - 1;
      while (j >= 0 && t[j] > key) { t[j + 1] = t[j]; j--; }
      t[j + 1] = key;
    }
    return (_n & 1) ? t[_n / 2] : 0.5f * (t[_n / 2 - 1] + t[_n / 2]);
  }
  float   _buf[N];
  uint8_t _n = 0, _idx = 0;
  float   _gate;
  float   _last = NAN;
};

/* ==========================================================================
 *  Calibration -- T_cal = gain * T_raw + offset
 *
 *  The MAX31850K already linearises the type-K curve and compensates the cold
 *  junction on-chip, so the residual error is a gentle affine offset/gain.
 *  Two reference points pin it down:
 *
 *    COLD : stirred ice-water bath  -> 0.00 C   (by definition)
 *    HOT  : rolling-boiling water   -> the LOCAL boiling point, NOT 100 C.
 *           boilingPointC() computes it from your ambient absolute pressure.
 *
 *  Refit whenever you change a probe, a lead, or the breakout.
 * ========================================================================== */
struct __attribute__((packed)) CalStore {
  uint32_t magic;
  uint8_t  version;
  float    gain[N_TEMP];
  float    offset[N_TEMP];
  float    rawCold[N_TEMP];   // averaged raw reading captured at the cold point
  float    rawHot[N_TEMP];    // averaged raw reading captured at the hot point
  float    refCold;           // reference temperature used for the cold point
  float    refHot;            // reference temperature used for the hot point
};
#define CAL_MAGIC    0x4F4C5454UL   // 'OLTT'
#define CAL_VERSION  1

CalStore cal;
bool     calValid = false;

/* Fixed physical slot -> ROM address map. Populate once with 'M' so a dropped
 * sensor can never shift calibration onto its neighbour. */
DeviceAddress slotAddr[N_TEMP];
bool          slotSet = false;

/* Per-slot state. */
SpikeFilter<MEDIAN_N> tempFilt[N_TEMP];
SpikeFilter<MEDIAN_N> pressFilt[N_PRESS];
float    lastGoodC[N_TEMP];
bool     haveGood[N_TEMP]  = { false };
uint32_t faultCount[N_TEMP] = { 0 };

/* Pressure calibration placeholders (identity until you fit them). */
float pressGain[N_PRESS]   = { 1.0f, 1.0f, 1.0f };
float pressOffset[N_PRESS] = { 0.0f, 0.0f, 0.0f };

/* ------------------------------------------------------------------ helpers */

/* True if a raw DallasTemperature reading is a fault code, not a measurement. */
static inline bool tcFault(float t) { return (t <= DEVICE_DISCONNECTED_C); }

/* Local boiling point of water from ambient ABSOLUTE pressure, hPa.
 * Clausius-Clapeyron. 1013.25 hPa -> 100.00 C. */
float boilingPointC(float p_hPa) {
  const float T0 = 373.15f;       // K at 1 atm
  const float P0 = 1013.25f;      // hPa at 1 atm
  const float dHvap = 40660.0f;   // J/mol, latent heat of vaporisation
  const float R = 8.314462f;      // J/mol/K
  float invT = 1.0f / T0 - (R / dHvap) * logf(p_hPa / P0);
  return 1.0f / invT - 273.15f;
}

int slotFor(const DeviceAddress a) {                 // ROM -> slot index, or -1
  if (!slotSet) return -1;
  for (uint8_t i = 0; i < N_TEMP; i++)
    if (memcmp(a, slotAddr[i], 8) == 0) return i;
  return -1;
}

void printAddr(const DeviceAddress a) {
  for (uint8_t i = 0; i < 8; i++) Serial.printf("%02X", a[i]);
}

void calLoad() {
  EEPROM.get(0, cal);
  calValid = (cal.magic == CAL_MAGIC && cal.version == CAL_VERSION);
  if (!calValid) {                                   // identity until calibrated
    cal.magic = CAL_MAGIC; cal.version = CAL_VERSION;
    for (uint8_t i = 0; i < N_TEMP; i++) { cal.gain[i] = 1.0f; cal.offset[i] = 0.0f; }
    cal.refCold = 0.0f; cal.refHot = 100.0f;
    Serial.println(F("No stored calibration -- running UNCALIBRATED (gain=1, off=0)."));
  }
}
void calSave() { EEPROM.put(0, cal); }

/* ------------------------------------------------------------------- reads */

/* Read every present device once; return each slot's FILTERED, CALIBRATED temp.
 * Faults hold last-good and bump a counter. If no slot map exists yet, fall back
 * to bus-index order so the logger still runs out of the box. */
void readTempsCalibrated(float* outC) {
  uint8_t n = oneWireSensors.getDeviceCount();
  bool seen[N_TEMP] = { false };

  for (uint8_t s = 0; s < n; s++) {
    DeviceAddress a; oneWireSensors.getAddress(a, s);
    int slot = slotFor(a);
    if (slot < 0) {                                  // no map -> use bus order
      if (!slotSet && s < N_TEMP) slot = s; else continue;
    }
    seen[slot] = true;

    float raw = oneWireSensors.getTempC(a);
    if (tcFault(raw)) {                              // FAULT: hold, count, report
      faultCount[slot]++;
      outC[slot] = haveGood[slot] ? lastGoodC[slot] : NAN;
      continue;
    }
    float m  = tempFilt[slot].update(raw);           // (1) remove spikes
    float tC = cal.gain[slot] * m + cal.offset[slot];// (2) apply calibration
    outC[slot] = tC;
    lastGoodC[slot] = tC; haveGood[slot] = true;
  }
  for (uint8_t i = 0; i < N_TEMP; i++)
    if (!seen[i]) outC[i] = haveGood[i] ? lastGoodC[i] : NAN;   // absent sensor
}

/* Average the RAW (unfiltered) reading per channel for `ms`. Used by capture. */
bool captureAverage(uint32_t ms, float* out) {
  float    sum[N_TEMP] = { 0 };
  uint16_t cnt[N_TEMP] = { 0 };
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    oneWireSensors.requestTemperatures();
    uint8_t n = oneWireSensors.getDeviceCount();
    for (uint8_t s = 0; s < n; s++) {
      DeviceAddress a; oneWireSensors.getAddress(a, s);
      int slot = slotFor(a);
      if (slot < 0) { if (!slotSet && s < N_TEMP) slot = s; else continue; }
      float raw = oneWireSensors.getTempC(a);
      if (!tcFault(raw)) { sum[slot] += raw; cnt[slot]++; }
    }
    delay(100);
  }
  bool ok = true;
  for (uint8_t i = 0; i < N_TEMP; i++) {
    if (cnt[i] == 0) { out[i] = NAN; ok = false; }
    else out[i] = sum[i] / cnt[i];
  }
  return ok;
}

/* Solve gain/offset from the two captured points and persist. */
void computeFit() {
  for (uint8_t i = 0; i < N_TEMP; i++) {
    float dr = cal.rawHot[i] - cal.rawCold[i];
    if (isnan(cal.rawHot[i]) || isnan(cal.rawCold[i]) || fabsf(dr) < 0.01f) {
      cal.gain[i] = 1.0f; cal.offset[i] = 0.0f;     // no span -> identity
      continue;
    }
    cal.gain[i]   = (cal.refHot - cal.refCold) / dr;
    cal.offset[i] = cal.refCold - cal.gain[i] * cal.rawCold[i];
  }
  calSave();
  calValid = true;
  Serial.println(F("Fit complete and saved to EEPROM."));
}

/* --------------------------------------------------------- serial console */

void printCal() {
  Serial.println(F("slot  addr              gain       offset(C)  rawCold    rawHot"));
  for (uint8_t i = 0; i < N_TEMP; i++) {
    Serial.printf("%2u    ", i);
    if (slotSet) printAddr(slotAddr[i]); else Serial.print(F("--"));
    Serial.printf("  %9.5f  %+8.3f   %8.3f   %8.3f\n",
                  cal.gain[i], cal.offset[i], cal.rawCold[i], cal.rawHot[i]);
  }
  Serial.printf("refCold=%.3f C   refHot=%.3f C   valid=%d\n",
                cal.refCold, cal.refHot, calValid);
}

void handleCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = toupper(cmd[0]);

  if (c == '?') {
    Serial.println(F("S            scan bus: print ROM addresses + live temps"));
    Serial.println(F("M            map current bus order into the 5 calibration slots"));
    Serial.println(F("C            capture COLD point (probes in stirred ICE bath, 0 C)"));
    Serial.println(F("H [p_hPa]    capture HOT point (boiling water). Optional ambient"));
    Serial.println(F("             absolute pressure in hPa; default 1013.25"));
    Serial.println(F("F            fit gain/offset from C+H and save to EEPROM"));
    Serial.println(F("P            print calibration table"));
    Serial.println(F("Z            zero calibration (gain=1, offset=0)"));
  }
  else if (c == 'S') {
    uint8_t n = oneWireSensors.getDeviceCount();
    oneWireSensors.requestTemperatures();
    Serial.printf("Found %u device(s):\n", n);
    for (uint8_t s = 0; s < n; s++) {
      DeviceAddress a; oneWireSensors.getAddress(a, s);
      Serial.printf("  %02X ", a[0]); printAddr(a);
      Serial.printf("  slot=%d  T=%3.3f C\n", slotFor(a), oneWireSensors.getTempC(a));
    }
  }
  else if (c == 'M') {
    uint8_t n = oneWireSensors.getDeviceCount();
    if (n != N_TEMP) { Serial.printf("Need %u devices, found %u.\n", N_TEMP, n); return; }
    for (uint8_t s = 0; s < n; s++) oneWireSensors.getAddress(slotAddr[s], s);
    slotSet = true;
    Serial.println(F("Mapped. Verify slot order matches the physical rig."));
    printCal();
  }
  else if (c == 'C') {
    Serial.println(F("Capturing COLD point (ice bath, 0.00 C)..."));
    if (captureAverage(CAL_CAPTURE_MS, cal.rawCold)) {
      cal.refCold = 0.0f;
      Serial.println(F("Cold captured."));
    } else Serial.println(F("Cold capture FAILED -- a channel was in fault."));
  }
  else if (c == 'H') {
    float p = 1013.25f;
    int sp = cmd.indexOf(' ');
    if (sp > 0) p = cmd.substring(sp + 1).toFloat();
    Serial.printf("Capturing HOT point; refHot = %.3f C at %.1f hPa...\n",
                  boilingPointC(p), p);
    if (captureAverage(CAL_CAPTURE_MS, cal.rawHot)) {
      cal.refHot = boilingPointC(p);
      Serial.println(F("Hot captured."));
    } else Serial.println(F("Hot capture FAILED -- a channel was in fault."));
  }
  else if (c == 'F') { computeFit(); printCal(); }
  else if (c == 'P') { printCal(); }
  else if (c == 'Z') {
    for (uint8_t i = 0; i < N_TEMP; i++) { cal.gain[i] = 1.0f; cal.offset[i] = 0.0f; }
    calSave(); Serial.println(F("Calibration zeroed."));
  }
}

/* --------------------------------------------------------------- lifecycle */

void blink() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}              // wait for USB, briefly
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  if (!mpr.begin(MPRLS_DEFAULT_ADDR, &Wire))
    Serial.println(F("MPRLS not responding -- check wiring."));

  oneWireSensors.begin();
  /* NOTE: no setResolution() here. The MAX31850K is fixed at 14-bit / 0.25 C;
   * calling setResolution() is a no-op and only invites confusion. */

  for (uint8_t i = 0; i < N_TEMP;  i++) tempFilt[i].setGate(SPIKE_GATE_C);
  for (uint8_t i = 0; i < N_PRESS; i++) pressFilt[i].setGate(SPIKE_GATE_KPA);

  calLoad();

  Serial.println(F("OLTT logger ready. Type ? for commands."));
  Serial.println(F("P1_kPa,P2_kPa,P3_kPa,T1_C,T2_C,T3_C,T4_C,T5_C,fault_mask"));
}

void loop() {
  /* --- serial console: calibration lives here --- */
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; }
    else if (line.length() < 64) line += ch;
  }

  /* --- 5 s cadence, non-blocking --- */
  static uint32_t nextLog = 0;
  if ((int32_t)(millis() - nextLog) < 0) return;
  nextLog += LOG_INTERVAL_MS;

  /* --- pressures: read -> fault-skip -> median (no EMA) -> cal --- */
  const uint8_t port[N_PRESS] = { 1, 2, 5 };
  float p[N_PRESS];
  for (uint8_t i = 0; i < N_PRESS; i++) {
    I2CMulti.selectPort(port[i]);
    delay(20);
    float raw = mpr.readPressure() / 10.0f;          // hPa -> kPa
    if (raw <= 0.0f) { p[i] = NAN; continue; }       // bus fault: don't filter it
    float m = pressFilt[i].update(raw);              // spikes removed
    p[i] = pressGain[i] * m + pressOffset[i];
  }

  /* --- temperatures: filtered + calibrated --- */
  oneWireSensors.requestTemperatures();
  float t[N_TEMP];
  readTempsCalibrated(t);

  /* --- emit one CSV line + a fault mask so wiring events are visible --- */
  Serial.printf("%4.2f,%4.2f,%4.2f,", p[0], p[1], p[2]);
  uint8_t faultMask = 0;
  for (uint8_t i = 0; i < N_TEMP; i++) {
    if (isnan(t[i])) Serial.print(F("nan,"));
    else             Serial.printf("%3.3f,", t[i]);
    if (isnan(t[i])) faultMask |= (1 << i);
  }
  Serial.printf("0x%02X\n", faultMask);

  blink();
}
