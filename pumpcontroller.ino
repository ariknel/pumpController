/*
 * Adaptive Cooling Pump Controller
 * Target: ATmega328P-AU (Arduino Uno / "ATmega328P" board profile), flashed via ISP
 * Author: Arik Nel
 *
 * Control priority (highest first):
 *   1. OVERHEAT   - NTC reads above TEMP_CRIT_C -> pump forced to 100%, whatever the mode
 *   2. MANUAL     - external dashboard pot connected -> pot sets the speed
 *   3. AUTO       - NTC connected -> speed follows the temperature curve
 *   4. FALLBACK   - no pot, no (valid) NTC -> pump runs at 100%
 *
 * Onboard trimpot = range trim for the external pot.
 *   The external pot is read as a rheostat against a pull-up, so its maximum ADC
 *   reading depends on its value (a 20k pot tops out lower than a 50k pot). The
 *   trimpot sets the ADC reading that counts as "full speed", so any pot value can
 *   use its whole rotation.
 *   Calibration: turn the dashboard pot fully up, then turn the trimpot until the
 *   status LED just reaches its fastest blink (100%). Done.
 *
 * Pinout:
 *   PD7 (D7)  - status LED
 *   PC0 (A0)  - NTC   (NTC to GND, NTC_SERIES_OHMS pull-up to 5V)
 *   PC1 (A1)  - onboard trimpot (wiper, ends on 5V/GND)
 *   PC2 (A2)  - external pot    (pot to GND, EXT_PULLUP pull-up to 5V; open = reads ~1023)
 *   PB1 (D9)  - MOSFET gate (OC1A, Timer1 hardware PWM)  <-- CHECK AGAINST SCHEMATIC
 */

#include <avr/wdt.h>

// ============================== CONFIGURATION ==============================

// --- Pins ---
const uint8_t PIN_LED     = 7;   // PD7
const uint8_t PIN_NTC     = A0;  // PC0
const uint8_t PIN_TRIM    = A1;  // PC1
const uint8_t PIN_EXTPOT  = A2;  // PC2
const uint8_t PIN_PWM     = 9;   // PB1 / OC1A - Timer1 is hard-wired to this pin

// --- PWM ---
const uint32_t PWM_FREQ_HZ = 20000;  // 20 kHz: above audible range, easy on the AON6354

// --- NTC thermistor ---
const float NTC_SERIES_OHMS = 10000.0;  // pull-up resistor value
const float NTC_R25_OHMS    = 10000.0;  // NTC resistance at 25 C
const float NTC_BETA        = 3950.0;   // NTC beta coefficient
const int   NTC_OPEN_ADC    = 1010;     // above this = sensor unplugged / broken wire
const int   NTC_SHORT_ADC   = 10;       // below this = sensor shorted

// --- Temperature curve (AUTO mode) ---
const float TEMP_START_C  = 50.0;  // below this: pump at IDLE_DUTY
const float TEMP_FULL_C   = 85.0;  // at/above this: pump at 100%
const float TEMP_CRIT_C   = 95.0;  // overheat: force 100% even in MANUAL mode
const float TEMP_CRIT_HYS = 5.0;   // overheat releases at TEMP_CRIT_C - TEMP_CRIT_HYS
const int   IDLE_DUTY     = 250;   // permille (25%) - keep some flow when cold, 0 = pump off

// --- External pot (MANUAL mode) ---
const int  EXT_OPEN_ADC      = 1000;  // reading above this = pot disconnected
const int  EXT_CONNECT_ADC   = 980;   // must drop below this to count as connected (hysteresis)
const int  EXT_FS_MIN_ADC    = 100;   // full-scale reading with trimpot fully CCW
const int  EXT_FS_MAX_ADC    = 975;   // full-scale reading with trimpot fully CW (< EXT_CONNECT_ADC)
const int  EXT_OFF_PERMILLE  = 30;    // bottom 3% of pot travel = pump off
const bool EXT_INVERT        = false; // true if the pot is wired so that CW lowers the reading

// --- Motor behaviour ---
const int      MIN_RUN_DUTY    = 150;  // permille - below this the pump stalls; used as floor when running
const int      RAMP_PER_TICK   = 20;   // max duty change per control tick (20 permille / 10 ms = 0->100% in 0.5 s)
const uint16_t KICKSTART_MS    = 300;  // full power burst when starting from standstill

// --- Timing ---
const uint16_t TICK_MS         = 10;   // control loop period
const uint16_t DETECT_DEBOUNCE = 20;   // ticks (200 ms) a connect/disconnect must persist

// --- Status LED ---
const uint32_t LED_MAX_HZ_X1000 = 8000;  // 8 Hz blink at 100% duty (blink rate scales with duty)
const uint32_t LED_MIN_HZ_X1000 = 500;   // never blink slower than 0.5 Hz while running
const uint16_t LED_IDLE_PERIOD  = 2000;  // pump off: short heartbeat flash every 2 s
const uint16_t LED_IDLE_FLASH   = 30;

// --- Debug output on UART (115200 baud) ---
#define DEBUG_SERIAL 0

// ============================================================================

enum Mode : uint8_t { MODE_FALLBACK, MODE_AUTO, MODE_MANUAL, MODE_OVERHEAT };

static uint16_t pwmTop;               // Timer1 TOP (ICR1)
static int      dutyOut = 0;          // current output duty, permille
static uint32_t kickUntil = 0;        // millis() until which kickstart is active

static float    ntcFilt, trimFilt, extFilt;  // low-pass filtered ADC readings

// Debounced "connected" flags
static bool     ntcOk = false, extOk = false;
static uint8_t  ntcCnt = 0, extCnt = 0;
static bool     overheat = false;

static uint32_t ledPhase = 0;         // 0..999999 = one blink cycle

// ----------------------------------------------------------------------------

// Averaged ADC read with a throwaway sample after switching the mux
// (lets the S/H cap settle on higher-impedance sources like a 50k pot).
static int readAdc(uint8_t pin) {
  analogRead(pin);
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) sum += analogRead(pin);
  return sum >> 3;
}

static void pwmInit() {
  pinMode(PIN_PWM, OUTPUT);
  digitalWrite(PIN_PWM, LOW);
  // Timer1: phase-correct PWM, TOP = ICR1 (mode 10), non-inverting on OC1A, no prescaler
  pwmTop = (uint16_t)(F_CPU / (2UL * PWM_FREQ_HZ));
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1   = pwmTop;
  OCR1A  = 0;
}

static void pwmWrite(int permille) {
  OCR1A = (uint16_t)((uint32_t)permille * pwmTop / 1000UL);
}

// Debounce a boolean presence signal
static void debounce(bool raw, bool &state, uint8_t &cnt) {
  if (raw == state) { cnt = 0; return; }
  if (++cnt >= DETECT_DEBOUNCE) { state = raw; cnt = 0; }
}

static float ntcToCelsius(float adc) {
  float r = NTC_SERIES_OHMS * adc / (1023.0 - adc);
  float invT = 1.0 / 298.15 + log(r / NTC_R25_OHMS) / NTC_BETA;
  return 1.0 / invT - 273.15;
}

static int tempCurve(float tC) {
  if (tC <= TEMP_START_C) return IDLE_DUTY;
  if (tC >= TEMP_FULL_C)  return 1000;
  float f = (tC - TEMP_START_C) / (TEMP_FULL_C - TEMP_START_C);
  int lo = max(IDLE_DUTY, MIN_RUN_DUTY);
  return lo + (int)(f * (1000 - lo));
}

// External pot position -> duty, scaled by the trimpot so any pot value reaches 100%
static int manualDuty(float ext, float trim) {
  float fullScale = EXT_FS_MIN_ADC + (trim / 1023.0) * (EXT_FS_MAX_ADC - EXT_FS_MIN_ADC);
  float pos = ext / fullScale;                 // 0.0 .. 1.0 (clamped below)
  if (pos > 1.0) pos = 1.0;
  if (EXT_INVERT) pos = 1.0 - pos;
  int p = (int)(pos * 1000.0);
  if (p <= EXT_OFF_PERMILLE) return 0;
  // Rescale remaining travel onto MIN_RUN..100% so the whole knob does something
  return MIN_RUN_DUTY + (long)(p - EXT_OFF_PERMILLE) * (1000 - MIN_RUN_DUTY) / (1000 - EXT_OFF_PERMILLE);
}

// Blink rate proportional to duty
static void updateLed(int duty, uint32_t now) {
  if (duty <= 0) {
    digitalWrite(PIN_LED, (now % LED_IDLE_PERIOD) < LED_IDLE_FLASH);
    return;
  }
  uint32_t hz1000 = LED_MAX_HZ_X1000 * (uint32_t)duty / 1000UL;
  if (hz1000 < LED_MIN_HZ_X1000) hz1000 = LED_MIN_HZ_X1000;
  // cycle = 1,000,000 units; per ms advance = Hz * 1e6 / 1000 = hz1000
  ledPhase = (ledPhase + hz1000 * TICK_MS) % 1000000UL;
  digitalWrite(PIN_LED, ledPhase < 500000UL);
}

// ----------------------------------------------------------------------------

void setup() {
  // Safe state first: pump output low until the timer takes over
  pwmInit();
  pinMode(PIN_LED, OUTPUT);

#if DEBUG_SERIAL
  Serial.begin(115200);
#endif

  // Prime filters with real readings so we don't start from zero
  ntcFilt  = readAdc(PIN_NTC);
  trimFilt = readAdc(PIN_TRIM);
  extFilt  = readAdc(PIN_EXTPOT);
  ntcOk = (ntcFilt < NTC_OPEN_ADC && ntcFilt > NTC_SHORT_ADC);
  extOk = (extFilt < EXT_CONNECT_ADC);

  wdt_enable(WDTO_500MS);
}

void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick = now;
  wdt_reset();

  // --- Read & filter inputs (EMA, ~80 ms time constant at 10 ms tick) ---
  ntcFilt  += (readAdc(PIN_NTC)    - ntcFilt)  * 0.125;
  trimFilt += (readAdc(PIN_TRIM)   - trimFilt) * 0.125;
  extFilt  += (readAdc(PIN_EXTPOT) - extFilt)  * 0.125;

  // --- Presence detection ---
  debounce(ntcFilt < NTC_OPEN_ADC && ntcFilt > NTC_SHORT_ADC, ntcOk, ntcCnt);
  bool extRaw = extOk ? (extFilt < EXT_OPEN_ADC) : (extFilt < EXT_CONNECT_ADC);
  debounce(extRaw, extOk, extCnt);

  float tempC = ntcOk ? ntcToCelsius(ntcFilt) : NAN;
  if (ntcOk) {
    if (tempC >= TEMP_CRIT_C) overheat = true;
    else if (tempC < TEMP_CRIT_C - TEMP_CRIT_HYS) overheat = false;
  } else {
    overheat = false;  // no sensor: FALLBACK/MANUAL decide
  }

  // --- Mode selection & target duty ---
  Mode mode;
  int target;
  if (overheat)      { mode = MODE_OVERHEAT; target = 1000; }
  else if (extOk)    { mode = MODE_MANUAL;   target = manualDuty(extFilt, trimFilt); }
  else if (ntcOk)    { mode = MODE_AUTO;     target = tempCurve(tempC); }
  else               { mode = MODE_FALLBACK; target = 1000; }

  // Enforce the stall floor for any non-zero request
  if (target > 0 && target < MIN_RUN_DUTY) target = MIN_RUN_DUTY;

  // --- Kickstart from standstill, then ramp ---
  if (dutyOut == 0 && target > 0) kickUntil = now + KICKSTART_MS;

  if (target > dutyOut)      dutyOut = min(target, dutyOut + RAMP_PER_TICK);
  else if (target < dutyOut) dutyOut = max(target, dutyOut - RAMP_PER_TICK);

  bool kicking = (int32_t)(kickUntil - now) > 0 && target > 0;
  pwmWrite(kicking ? 1000 : dutyOut);

  updateLed(dutyOut, now);

#if DEBUG_SERIAL
  static uint8_t dbg = 0;
  if (++dbg >= 50) {  // every 500 ms
    dbg = 0;
    static const char *const names[] = { "FALLBACK", "AUTO", "MANUAL", "OVERHEAT" };
    Serial.print(names[mode]);
    Serial.print(F("  T="));    Serial.print(tempC, 1);
    Serial.print(F("C ntc="));  Serial.print((int)ntcFilt);
    Serial.print(F(" ext="));   Serial.print((int)extFilt);
    Serial.print(F(" trim="));  Serial.print((int)trimFilt);
    Serial.print(F(" duty="));  Serial.print(dutyOut / 10);
    Serial.println('%');
  }
#else
  (void)mode;
#endif
}
