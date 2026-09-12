/*
  ============================================================================
  ESC TEST BENCH  —  SparkFun RedBoard Turbo  +  AM32 ESC (FreelyRC V2)
                      Motor: QWinout 4108 580KV
  ============================================================================

  WHAT THIS DOES
  --------------
  Drives one AM32-flashed ESC with a standard hobby PWM (servo-style) signal
  and gives you a serial-terminal interface to arm, disarm, set throttle
  (0-100%), and — if your ESC is configured for it — reverse direction.

  ----------------------------------------------------------------------------
  !! SAFETY — READ BEFORE POWERING ANYTHING !!
  ----------------------------------------------------------------------------
   - REMOVE THE PROPELLER (or any attached load) for all initial bench tests.
   - Secure/clamp the motor — it can walk or flip when it spins up.
   - The QWinout 4108 580KV is rated up to ~37A continuous / 3-6S. Confirm
     your FreelyRC V2 ESC's continuous current rating meets or exceeds
     whatever you actually plan to draw, and use a battery/PSU that can
     supply it. When in doubt, test briefly at low throttle first.
   - Keep hands, hair, wires and tools clear of the shaft at all times.
   - Wire ESC signal ground to RedBoard ground (common ground is required).
     Do NOT connect the ESC's red BEC wire to the RedBoard 5V pin unless you
     intend to power the board FROM the ESC's BEC — don't back-feed two
     power sources into each other. Simplest setup: power the RedBoard from
     USB, and connect only the ESC's signal + ground wires to it.
   - Power the ESC's main battery input AFTER you've uploaded this sketch
     and have the SerialUSB Monitor open. AM32 (like most ESC firmware) arms
     by seeing a valid low-throttle signal right as it boots — this sketch
     starts sending minimum throttle the instant it starts running, so the
     ESC should see it from power-up.

  ----------------------------------------------------------------------------
  WIRING
  ----------------------------------------------------------------------------
   ESC signal wire  -> RedBoard Turbo pin 5   (change ESC_PIN below if needed)
   ESC signal ground -> RedBoard Turbo GND
   ESC battery leads -> your LiPo / bench PSU (matched to motor + ESC ratings)
   ESC 3 motor wires -> QWinout 4108 motor's 3 phase wires (any order to
                         start; swapping any two reverses rotation — this is
                         the simplest, foolproof way to fix rotation
                         direction if you don't need it reversible live)

  ----------------------------------------------------------------------------
  DIRECTION / REVERSE — IMPORTANT
  ----------------------------------------------------------------------------
  A plain PWM signal only tells a "normal mode" AM32 ESC how fast to spin —
  not which way. Live, software-commanded direction reversal requires
  "Bidirectional Mode" (a.k.a. 3D mode) to be turned ON in the ESC itself
  first, using the AM32 web configurator (https://am32.ca or the
  Betaflight-passthrough config tool) BEFORE this sketch can do anything
  useful with it. That is a one-time ESC setting change, not something this
  sketch can do over the signal wire.

    - If Bidirectional Mode is OFF in the ESC (the AM32 default), leave
      BIDIRECTIONAL_MODE below set to false. You'll get 0-100% throttle in
      one direction; to reverse, power everything down and swap two of the
      three motor phase wires.

    - If you've enabled Bidirectional Mode in the ESC, set
      BIDIRECTIONAL_MODE to true below. Throttle range becomes -100% (full
      reverse) to +100% (full forward), with 0% = stopped, centered on a
      1500us neutral pulse.

  ----------------------------------------------------------------------------
  SERIAL COMMANDS  (SerialUSB Monitor: 115200 baud, line ending = "Newline")
  ----------------------------------------------------------------------------
    a            arm the ESC (hold at idle ~3s, per AM32 arming sequence)
    d  or  x     disarm / immediate stop (hard cut to idle, safest "STOP")
    s <n>        set throttle target to n percent
                   unidirectional: 0 to 100
                   bidirectional : -100 to 100 (negative = reverse)
    0            ramp throttle down to 0% (stays armed)
    +            increase throttle by THROTTLE_STEP percent
    -            decrease throttle by THROTTLE_STEP percent
    f            quick "forward" at last-used magnitude (bidirectional only)
    r            quick "reverse" at last-used magnitude (bidirectional only)
    p            print current status
    ?  or  h     print this help / command list
  ============================================================================
*/

#include <Servo.h>

// ---------------------------- USER CONFIGURATION ---------------------------
const uint8_t  ESC_PIN            = 5;      // signal pin to the ESC
const bool     BIDIRECTIONAL_MODE = true;  // true ONLY if ESC's Bidirectional
                                             // Mode has been enabled in the
                                             // AM32 configurator (see notes above)

const uint16_t PULSE_MIN_US       = 1000;   // idle / full reverse
const uint16_t PULSE_MAX_US       = 2000;   // full throttle / full forward
const uint16_t PULSE_NEUTRAL_US   = 1500;   // bidirectional stop point
const uint16_t PULSE_DEADBAND_US  = 25;     // small deadband around neutral

const unsigned long ARM_HOLD_MS   = 3000;   // time to hold idle signal to arm
const int      THROTTLE_STEP      = 5;      // percent per +/- command
const float    RAMP_RATE_PCT_SEC  = 80.0;   // max percent/second throttle change
// -----------------------------------------------------------------------------

Servo esc;

enum State { DISARMED, ARMING, ARMED };
State state = DISARMED;

int   targetPercent  = 0;     // where the user wants the throttle
float outputPercent  = 0.0;   // where the throttle actually is (ramped)
int   lastMagnitude  = 20;    // used by the quick f/r commands
unsigned long armStartTime  = 0;
unsigned long lastRampMillis = 0;

// ---------------------------------------------------------------------------
void setup() {
  SerialUSB.begin(115200);
  unsigned long t0 = millis();
  while (!SerialUSB && millis() - t0 < 3000) { /* wait briefly for USB CDC */ }

  pinMode(LED_BUILTIN, OUTPUT);

  esc.attach(ESC_PIN, PULSE_MIN_US, PULSE_MAX_US);
  writePulseForPercent(0);   // safe idle signal immediately, before anything else

  lastRampMillis = millis();
  printHelp();
  SerialUSB.println(F("\nESC idling at minimum signal. Power on the ESC now if you haven't."));
  SerialUSB.println(F("Type 'a' to begin arming.\n"));
}

// ---------------------------------------------------------------------------
void loop() {
  handleSerialUSB();
  updateArming();
  updateRamp();
  updateStatusLED();
}

// ---------------------------------------------------------------------------
// Convert a -100..100 (or 0..100) percent value into a pulse width and send it
void writePulseForPercent(float percent) {
  uint16_t us;
  if (BIDIRECTIONAL_MODE) {
    percent = constrain(percent, -100.0, 100.0);
    if (fabs(percent) < 1.0) {
      us = PULSE_NEUTRAL_US;
    } else if (percent > 0) {
      us = map((long)percent, 0, 100, PULSE_NEUTRAL_US + PULSE_DEADBAND_US, PULSE_MAX_US);
    } else {
      us = map((long)percent, 0, -100, PULSE_NEUTRAL_US - PULSE_DEADBAND_US, PULSE_MIN_US);
    }
  } else {
    percent = constrain(percent, 0.0, 100.0);
    us = map((long)percent, 0, 100, PULSE_MIN_US, PULSE_MAX_US);
  }
  us = constrain(us, PULSE_MIN_US, PULSE_MAX_US);
  esc.writeMicroseconds(us);
}

// ---------------------------------------------------------------------------
void startArming() {
  if (state == ARMED) {
    SerialUSB.println(F("Already armed."));
    return;
  }
  SerialUSB.println(F("Arming... keep clear, throttle held at idle."));
  targetPercent  = 0;
  outputPercent  = 0;
  writePulseForPercent(0);
  state = ARMING;
  armStartTime = millis();
}

void updateArming() {
  if (state == ARMING && millis() - armStartTime >= ARM_HOLD_MS) {
    state = ARMED;
    SerialUSB.println(F("Armed. Motor ready — use 's <percent>' or +/- to set speed."));
  }
}

// Immediate hard stop + disarm — this is the "emergency stop"
void disarm() {
  targetPercent = 0;
  outputPercent = 0;
  writePulseForPercent(0);
  state = DISARMED;
  SerialUSB.println(F("DISARMED. Motor stopped."));
}

// ---------------------------------------------------------------------------
void setThrottlePercent(int percent) {
  if (state != ARMED) {
    SerialUSB.println(F("Not armed. Send 'a' first."));
    return;
  }
  if (!BIDIRECTIONAL_MODE && percent < 0) {
    SerialUSB.println(F("Reverse needs Bidirectional Mode enabled in the AM32"));
    SerialUSB.println(F("configurator AND BIDIRECTIONAL_MODE = true in this sketch."));
    percent = 0;
  }
  int lo = BIDIRECTIONAL_MODE ? -100 : 0;
  percent = constrain(percent, lo, 100);
  targetPercent = percent;
  if (percent != 0) lastMagnitude = abs(percent);

  SerialUSB.print(F("Target throttle: "));
  SerialUSB.print(percent);
  SerialUSB.println(F("%"));
}

// Smoothly move outputPercent toward targetPercent so throttle changes aren't
// instantaneous current spikes; runs every loop() iteration.
void updateRamp() {
  unsigned long now = millis();
  float dt = (now - lastRampMillis) / 1000.0;
  lastRampMillis = now;

  if (state != ARMED) {
    outputPercent = 0;
    return;
  }

  float diff = targetPercent - outputPercent;
  float maxStep = RAMP_RATE_PCT_SEC * dt;
  if (fabs(diff) <= maxStep) {
    outputPercent = targetPercent;
  } else {
    outputPercent += (diff > 0 ? maxStep : -maxStep);
  }
  writePulseForPercent(outputPercent);
}

// ---------------------------------------------------------------------------
void handleSerialUSB() {
  if (!SerialUSB.available()) return;
  String line = SerialUSB.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  String rest = line.substring(1);
  rest.trim();

  switch (cmd) {
    case 'a': case 'A':
      startArming();
      break;

    case 'd': case 'D': case 'x': case 'X':
      disarm();
      break;

    case 's': case 'S':
      setThrottlePercent(rest.toInt());
      break;

    case '0':
      setThrottlePercent(0);
      break;

    case '+':
      setThrottlePercent(targetPercent + THROTTLE_STEP);
      break;

    case '-':
      setThrottlePercent(targetPercent - THROTTLE_STEP);
      break;

    case 'f': case 'F':
      if (!BIDIRECTIONAL_MODE) {
        SerialUSB.println(F("Forward is the only direction available in unidirectional mode."));
      } else {
        setThrottlePercent(lastMagnitude);
      }
      break;

    case 'r': case 'R':
      if (!BIDIRECTIONAL_MODE) {
        SerialUSB.println(F("Reverse needs Bidirectional Mode set in the ESC + sketch (see header notes)."));
      } else {
        setThrottlePercent(-lastMagnitude);
      }
      break;

    case 'p': case 'P':
      printStatus();
      break;

    case '?': case 'h': case 'H':
      printHelp();
      break;

    default:
      SerialUSB.println(F("Unknown command. Type '?' for help."));
  }
}

// ---------------------------------------------------------------------------
void printStatus() {
  SerialUSB.print(F("State: "));
  switch (state) {
    case DISARMED: SerialUSB.print(F("DISARMED")); break;
    case ARMING:   SerialUSB.print(F("ARMING"));   break;
    case ARMED:    SerialUSB.print(F("ARMED"));    break;
  }
  SerialUSB.print(F("  | Target: "));
  SerialUSB.print(targetPercent);
  SerialUSB.print(F("%  | Actual: "));
  SerialUSB.print(outputPercent, 1);
  SerialUSB.print(F("%  | Mode: "));
  SerialUSB.println(BIDIRECTIONAL_MODE ? F("BIDIRECTIONAL") : F("UNIDIRECTIONAL"));
}

void printHelp() {
  SerialUSB.println(F("--------------------------------------------------------"));
  SerialUSB.println(F(" AM32 ESC Test Bench - RedBoard Turbo"));
  SerialUSB.println(F("--------------------------------------------------------"));
  SerialUSB.println(F(" a        arm ESC"));
  SerialUSB.println(F(" d / x    disarm / emergency stop"));
  SerialUSB.println(F(" s <n>    set throttle percent (see mode below)"));
  SerialUSB.println(F(" 0        ramp to 0% (stays armed)"));
  SerialUSB.println(F(" +  -     step throttle up/down"));
  SerialUSB.println(F(" f  r     quick forward / reverse (bidirectional only)"));
  SerialUSB.println(F(" p        print status"));
  SerialUSB.println(F(" ?  h     this help"));
  SerialUSB.print(F(" Mode: "));
  SerialUSB.println(BIDIRECTIONAL_MODE
    ? F("BIDIRECTIONAL  (s accepts -100..100)")
    : F("UNIDIRECTIONAL (s accepts 0..100; reverse = swap 2 motor wires)"));
  SerialUSB.println(F("--------------------------------------------------------"));
}

// ---------------------------------------------------------------------------
// Onboard LED: off = disarmed, fast blink = arming, slow blink = armed/idle,
// solid on = armed and spinning.
void updateStatusLED() {
  unsigned long now = millis();
  switch (state) {
    case DISARMED:
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case ARMING:
      digitalWrite(LED_BUILTIN, (now / 150) % 2);
      break;
    case ARMED:
      if (fabs(outputPercent) < 1.0) {
        digitalWrite(LED_BUILTIN, (now / 600) % 2);
      } else {
        digitalWrite(LED_BUILTIN, HIGH);
      }
      break;
  }
}
