


/*Beta 2.0


  v2.0 now included latchable launch control – Stable shift state machine - ratio learing can be toggled on and off - rev match gain adjustable gain per gear


  NEW lauch conntrol hold down both shift paddles while sationary for 3 seconds, rev counter will count up 123 once toggled on rev counter does 2
  needle sweeps, if toggled off rev counter does 1 needle sweep
  Continuous rev-match
  Latched target gear
  Torque ramp
  Continuous road-speed tracking
  Neutral freeze for gear calculation
  Extended diagnostics (LG, TG, TR, ERR, MERR, BT, RT, CAN)
  Ratio-confirm completion (completion based on fabs(Ratiocalc - shiftTargetRatio) < SHIFT_RATIO_TOLERANCE)
  added high speed data logging
  launch control (traction control) default off, tunable by gear,

  Important note, Open inverter cruise control speed filter must be set to 0 and gain must be tuned control the spinning mass of just the motor
  in nuetral before shift gain tuning can commence, if internting you use cruisee control to drive the whole car
  at a stedy state you will have to impiment some way of changing the gain back and forwards via can for each use case, i wont explore here.

*/


// ============================================================
// RX-8 EV VCU
// ============================================================


// ============================================================
// LIBRARIES
// ============================================================

#include <Arduino.h>
#include <Arduino_CAN.h>

#include <CanUtil.h>
#include <R7FA4M1_CAN.h>
#include <R7FA6M5_CAN.h>
#include <SyncCanMsgRingbuffer.h>

#include "serial_console.h"
#include "vcu_settings.h"


// ============================================================
// HARDWARE / INPUT STRUCTURES
// ============================================================

struct DebounceInput {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeTime;
};


// ============================================================
// CONSTANTS
// ============================================================

// ------------------------------------------------------------
// CAN IDs
// ------------------------------------------------------------

#define INVERTER_CMD_ID 0x3F


// ------------------------------------------------------------
// Input / button timing
// ------------------------------------------------------------

#define DEBOUNCE_TIME 15


// ------------------------------------------------------------
// Gear / shift timing
// ------------------------------------------------------------

#define SHIFT_MAX_TIME 3000              // ms - safety timeout
#define GEAR_CONFIRM_TIME 50
#define GEAR_ENGAGED_CONFIRM_TIME 75
#define NEUTRAL_EXIT_CONFIRM_TIME 1000

#define SHIFT_RATIO_TOLERANCE 0.050f
#define GEAR_HYSTERESIS 0.15


// ------------------------------------------------------------
// Traction control
// ------------------------------------------------------------

#define TC_TACH_START_DELAY 1050
#define TC_TACH_LOW_TIME    525
#define TC_TACH_HIGH_TIME   1050


#define TC_TARGET_SLIP_PERCENT 7.0f
#define TC_SLIP_HYSTERESIS 1.5f
#define TC_LOW_SPEED_KMH 5.0f
#define TC_LOW_SPEED_SLIP_KMH 1.0f
#define TC_MIN_THROTTLE_PERCENT 80


// ------------------------------------------------------------
// Throttle
// ------------------------------------------------------------

#define THROTTLE_IDLE_THRESHOLD 100
#define THROTTLE_FAULT_CLEAR_TIME 1000     // ms
#define THROTTLE_RAW_MARGIN 40


// ============================================================
// PIN DEFINITIONS
// ============================================================

// ------------------------------------------------------------
// Digital inputs
// ------------------------------------------------------------

const int UP_pin = 7;
const int DOWN_pin = 8;
const int GearN_pin = 9;
const int brakeSwitchPin = 12;


// ------------------------------------------------------------
// Digital 12V isolated inputs
// ------------------------------------------------------------

//const int optoInput1 = 2;
//const int optoInput2 = 3;


// ------------------------------------------------------------
// Analogue inputs
// ------------------------------------------------------------

const int throttle1Pin = A4;
const int throttle2Pin = A3;
const int regenPin = A2;

//const int spare analog = A1;
//const int spare analog = A0;


// ------------------------------------------------------------
// Outputs
// ------------------------------------------------------------

const int Oil_pump = 13;

//const int out2 = 10;
//const int out3 = 11;
//const int out4 = 6;


// ============================================================
// BUTTON / INPUT STATE
// ============================================================

DebounceInput upBtn =
  { UP_pin, LOW, LOW, 0 };

DebounceInput downBtn =
  { DOWN_pin, LOW, LOW, 0 };

DebounceInput gearNBtn =
  { GearN_pin, LOW, LOW, 0 };


bool upButton = false;
bool downButton = false;
bool neutralButton = false;

bool brakePressed = false;
bool clutchPressed = false;
bool ignitionOn = false;
bool reverseSelected = false;



// ============================================================
// TIMING
// ============================================================

// ------------------------------------------------------------
// Main task timers
// ------------------------------------------------------------

unsigned long lastInverterUpdate = 0;
unsigned long lastPCMUpdate = 0;
unsigned long lastDebugUpdate = 0;
unsigned long lastLog = 0;
unsigned long lastEngineRun = 0;


// ------------------------------------------------------------
// Heartbeat
// ------------------------------------------------------------

unsigned long lastHeartbeat = 0;
bool heartbeatState = false;


// ------------------------------------------------------------
// Gear / shift timers
// ------------------------------------------------------------

unsigned long shiftStartTime = 0;
unsigned long gearEngagedTimer = 0;
unsigned long targetGearTimer = 0;

bool gearEngagedSeen = false;


// ------------------------------------------------------------
// Odometer timing
// ------------------------------------------------------------

unsigned long ODORefreshTime = 0;


// ============================================================
// CAN DIAGNOSTICS
// ============================================================

uint32_t canTxCount = 0;
uint32_t canRxCount = 0;

uint32_t lastTxID = 0;
uint32_t lastRxID = 0;

uint32_t lastCanDebugTime = 0;

uint32_t txPerSecond = 0;
uint32_t rxPerSecond = 0;

uint32_t lastTxSnapshot = 0;
uint32_t lastRxSnapshot = 0;

uint32_t pcmOverruns = 0;

bool canDumpEnabled = false;


// ============================================================
// ODOMETER
// ============================================================

long ODOus = 4500000;


// ============================================================
// INVERTER STATE
// ============================================================

uint8_t canrun1 = 0;
uint8_t canrun2 = 0;

uint16_t cruiseTarget = 0;
uint8_t regenPreset = 0;

bool inverterSeen = false;
unsigned long lastInverterMessage = 0;


// ============================================================
// RX-8 PCM / DASHBOARD
// ============================================================

enum PCMBurstState {
  PCM_IDLE,
  PCM_203,
  PCM_215,
  PCM_231,
  PCM_240,
  PCM_620,
  PCM_630,
  PCM_650,
  PCM_201
};

PCMBurstState pcmState = PCM_IDLE;

unsigned long pcmNextFrameTime = 0;


// ------------------------------------------------------------
// PCM transmit buffers
// ------------------------------------------------------------

uint8_t send420[7] =
  { 0, 0, 0, 0, 0, 0, 0 };

uint8_t send203[7] =
  { 19, 19, 19, 19, 175, 3, 19 };

uint8_t send215[8] =
  { 2, 45, 2, 45, 2, 42, 6, 129 };

uint8_t send231[8] =
  { 15, 0, 255, 255, 0, 0, 0, 0 };

uint8_t send240[8] =
  { 4, 0, 40, 0, 2, 55, 6, 129 };

uint8_t send620[7] =
  { 0, 0, 0, 0, 0, 0, 4 };

uint8_t send630[8] =
  { 8, 0, 0, 0, 0, 0, 106, 106 };

uint8_t send650[1] =
  { 0 };

uint8_t send201[8] =
  { 0, 0, 255, 255, 0, 0, 0, 255 };


// ------------------------------------------------------------
// Vehicle / motor display values
// ------------------------------------------------------------

int engTemp = 0;
bool oilPressure = true;

int engineRPM = 0;          // Scaled for RX-8 dashboard
int motorRPM = 0;           // Actual inverter / motor RPM

int vehicleSpeed = 0;
int transmissionSpeed = 0;


// ============================================================
// DRIVETRAIN
// ============================================================

const float TYRE_CIRCUMFERENCE_M = 2.082f;  // 225/40 R19
const float FINAL_DRIVE_RATIO = 4.3f;


// ------------------------------------------------------------
// Gear ratios
// ------------------------------------------------------------

float learnedRatio[6] = {
  0,
  3.684,
  2.155,
  1.569,
  1.057,
  0.805
};


// ------------------------------------------------------------
// Gear torque gains
// ------------------------------------------------------------

float gearGain[6] = {
  1.000,
  1.000,
  1.000,
  1.000,
  1.000,
  1.000
};


// ============================================================
// GEAR SHIFT STATE MACHINE
// ============================================================

enum ShiftState {
  SHIFT_IDLE,
  SHIFT_TORQUE_CUT,
  SHIFT_REV_MATCH
};

enum ShiftDirection {
  SHIFT_NONE,
  SHIFT_UP,
  SHIFT_DOWN
};


ShiftState shiftState = SHIFT_IDLE;
ShiftDirection shiftDirection = SHIFT_NONE;


// ------------------------------------------------------------
// Current / target gear
// ------------------------------------------------------------

int Gear = 0;
int latchedGear = 0;
int shiftTargetGear = 0;


// ------------------------------------------------------------
// Shift calculations
// ------------------------------------------------------------

float shiftTargetRatio = 0.0f;

uint16_t shiftBaseTarget = 0;
uint16_t shiftCruiseTarget = 0;

bool targetGearSeen = false;
bool shiftTorqueCut = false;
bool shiftCruiseEnable = false;


// ------------------------------------------------------------
// Shift torque control
// ------------------------------------------------------------

uint8_t shiftTorquePercent = 100;

// Percentage change per 10 ms inverter command
uint8_t SHIFT_TORQUE_RAMP_DOWN = 20;
uint8_t SHIFT_TORQUE_RAMP_UP = 10;


// ------------------------------------------------------------
// Shift / ratio calculations
// ------------------------------------------------------------

float Ratioact = 0;
float Ratiocalc = 0;

float wheelRPM = 0;
float gearboxOutputRPM = 0;

bool lastUpState = false;
bool lastDownState = false;


// ============================================================
// GEAR RATIO LEARNING
// ============================================================

float ratioSum[6] = { 0.0f };
uint32_t ratioCount[6] = { 0 };
float ratioAverage[6] = { 0.0f };

bool ratioLearningActive = false;
bool ratioLearningEnabled = false;
bool steadyState = false;

int learnGear = 0;
int lastLearnGear = 0;

unsigned long neutralExitTimer = 0;
unsigned long gearStableTimer = 0;
unsigned long steadyStateTimer = 0;

int lastMotorRPM = 0;
int lastTransmissionSpeed = 0;

float lastRatioCalc = 0.0f;

// ============================================================
// TRACTION CONTROL
// ============================================================

// ------------------------------------------------------------
// PID gains
// ------------------------------------------------------------
// Kp = torque reduction per % slip error
// Ki = torque reduction per second of accumulated slip error
// Kd = torque reduction per % slip error / second
//
// These are deliberately global so they can be tuned from
// the serial console during testing.
// ------------------------------------------------------------

float TC_KP[6] = {
  0.0f,
  2.0f,   // 1st
  2.0f,   // 2nd
  2.0f,   // 3rd
  2.0f,   // 4th
  2.0f    // 5th
};

float TC_KI[6] = {
  0.0f,
  0.20f,  // 1st
  0.20f,  // 2nd
  0.20f,  // 3rd
  0.20f,  // 4th
  0.20f   // 5th
};

float TC_KD[6] = {
  0.0f,
  0.05f,  // 1st
  0.05f,  // 2nd
  0.05f,  // 3rd
  0.05f,  // 4th
  0.05f   // 5th
};


// ------------------------------------------------------------
// TC state
// ------------------------------------------------------------

bool tractionControlEnabled = false;
bool tractionControlActive = false;

uint8_t tcTorquePercent = 100;


// ------------------------------------------------------------
// PID state
// ------------------------------------------------------------

float tcIntegral = 0.0f;
float tcPreviousError = 0.0f;

unsigned long tcLastUpdate = 0;


// ------------------------------------------------------------
// TC output slew limiting
// ------------------------------------------------------------
// Maximum torque percentage movement per second.
// This provides a final safety layer around the PID output.
// ------------------------------------------------------------

float TC_MAX_TORQUE_DOWN_PER_SEC = 200.0f;
float TC_MAX_TORQUE_UP_PER_SEC   = 50.0f;

// ------------------------------------------------------------
// Wheel speed / slip
// ------------------------------------------------------------

float frontWheelSpeedKmh = 0.0f;
float rearWheelSpeedKmh = 0.0f;

float wheelSlipKmh = 0.0f;
float wheelSlipPercent = 0.0f;


// ------------------------------------------------------------
// TC button / tachometer confirmation
// ------------------------------------------------------------

unsigned long tcButtonHoldStart = 0;
bool tcToggleHandled = false;

bool tcTachOverride = false;
unsigned long tcTachStartTime = 0;
uint8_t tcTachSweepCount = 0;


// ============================================================
// THROTTLE / REGEN
// ============================================================

uint16_t throttleMaxDiff = 300;

bool invertThrottle1 = false;
bool invertThrottle2 = false;

uint16_t throttle1Min = 323;
uint16_t throttle1Max = 794;

uint16_t throttle2Min = 212;
uint16_t throttle2Max = 683;


// ------------------------------------------------------------
// Throttle readings
// ------------------------------------------------------------

uint16_t throttle1_raw = 0;
uint16_t throttle2_raw = 0;

uint16_t throttle1_scaled = 0;
uint16_t throttle2_scaled = 0;

uint16_t throttle_diff = 0;


// ------------------------------------------------------------
// Throttle faults
// ------------------------------------------------------------

bool throttleFault = false;
bool throttleRangeFault = false;

unsigned long throttleFaultClearTimer = 0;


// ------------------------------------------------------------
// Regen
// ------------------------------------------------------------

bool invertRegen = false;

uint16_t regen_raw = 0;
uint16_t regen_scaled = 0;


// ------------------------------------------------------------
// Debug
// ------------------------------------------------------------

bool debugEnabled = false;


// ============================================================
// DATA LOGGING
// ============================================================

bool loggingEnabled = false;


// ============================================================
// LOW-LEVEL FUNCTIONS
// ============================================================

// ------------------------------------------------------------
// Button debounce
// ------------------------------------------------------------

void updateDebounce(DebounceInput &input)
{
  bool reading = digitalRead(input.pin);

  if (reading != input.lastReading)
    input.lastChangeTime = millis();

  if ((millis() - input.lastChangeTime) > DEBOUNCE_TIME)
    input.stableState = reading;

  input.lastReading = reading;
}

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

inline void sendFrame(uint16_t id, uint8_t dlc, uint8_t *data);


// ------------------------------------------------------------
// Odometer calculation
// ------------------------------------------------------------

long calcMicrosecODO(float speedKMH)
{
  float speedMPH = speedKMH / 160.934;
  float freq = speedMPH * 1.15;

  if (freq <= 0)
    return 4500000;

  long uS = 1000000 / freq;

  if (uS < 4500000)
    return uS;

  return 4500000;
}


// ------------------------------------------------------------
// OpenInverter CRC
// ------------------------------------------------------------

uint8_t computeOpenInverterCRC(uint32_t *data)
{
  uint32_t crc = 0xFFFFFFFF;

  for (int w = 0; w < 2; w++)
  {
    crc ^= data[w];

    for (int i = 0; i < 32; i++)
    {
      if (crc & 0x80000000)
        crc = (crc << 1) ^ 0x04C11DB7;
      else
        crc <<= 1;
    }
  }

  return crc & 0xFF;
}


// --------------------------------------------------
// Gear Calculation
// --------------------------------------------------
void calcGear() {
  if ((motorRPM < 50) || (transmissionSpeed < 5))
    return;

  // Convert rear road speed (km/h) to wheel RPM
  wheelRPM =
    ((float)transmissionSpeed * 1000.0f) / (TYRE_CIRCUMFERENCE_M * 60.0f);

  // Convert wheel RPM to gearbox output shaft RPM
  gearboxOutputRPM =
    wheelRPM * FINAL_DRIVE_RATIO;

  // Calculate actual gearbox ratio
  Ratiocalc =
    (float)motorRPM / gearboxOutputRPM;

  // Find closest gear
  int detectedGear = 0;
  float smallestError = 999.0;

  for (int i = 1; i <= 5; i++) {
    float error = abs(Ratiocalc - learnedRatio[i]);

    if (error < smallestError) {
      smallestError = error;
      detectedGear = i;
    }
  }

  // Apply hysteresis
  if (Gear == 0) {
    Gear = detectedGear;
  } else {
    float currentError = abs(Ratiocalc - learnedRatio[Gear]);
    float newError = abs(Ratiocalc - learnedRatio[detectedGear]);

    if (newError + GEAR_HYSTERESIS < currentError) {
      Gear = detectedGear;
    }
  }

  // Update active ratio for debug/other use
  Ratioact = learnedRatio[Gear];
}

void abortShift() {
  shiftDirection = SHIFT_NONE;
  shiftTargetGear = 0;
  shiftTargetRatio = 0;
  shiftBaseTarget = 0;
  shiftCruiseTarget = 0;
  targetGearTimer = 0;
  latchedGear = 0;
  shiftTorquePercent = 100;
  shiftTorqueCut = false;
  shiftCruiseEnable = false;
  gearEngagedSeen = false;
  gearEngagedTimer = 0;
  neutralExitTimer = 0;

  shiftState = SHIFT_IDLE;
  targetGearSeen = false;
}


//
void updateShiftLogic() {
  shiftTorqueCut = false;
  shiftCruiseEnable = false;
  shiftCruiseTarget = 0;

  // -----------------------------
  // Overall shift safety timeout
  // -----------------------------
  if (shiftState != SHIFT_IDLE && millis() - shiftStartTime >= SHIFT_MAX_TIME) {
    abortShift();
    return;
  }

  switch (shiftState) {
    case SHIFT_IDLE:
      break;

    case SHIFT_TORQUE_CUT:
      {
        shiftTorqueCut = true;


        if (gearNBtn.stableState && shiftTorquePercent == 0) {
          shiftState = SHIFT_REV_MATCH;
        }

        break;
      }


    case SHIFT_REV_MATCH:
      {
        shiftTorqueCut = true;
        shiftCruiseEnable = true;

        // ------------------------------------
        // Continuously calculate target RPM
        // from road speed
        // ------------------------------------

        wheelRPM =
          ((float)transmissionSpeed * 1000.0f) / (TYRE_CIRCUMFERENCE_M * 60.0f);

        gearboxOutputRPM =
          wheelRPM * FINAL_DRIVE_RATIO;

        shiftBaseTarget =
          shiftTargetRatio * gearboxOutputRPM;


        shiftCruiseTarget =
          shiftBaseTarget * gearGain[shiftTargetGear];

        // ------------------------------------
        // Still in neutral
        // Keep rev matching but don't attempt
        // gear detection.
        // ------------------------------------

        if (gearNBtn.stableState) {
          neutralExitTimer = 0;
          targetGearSeen = false;
          targetGearTimer = 0;
          break;
        }

        // ------------------------------------
        // First time we've left neutral
        // ------------------------------------



        if (neutralExitTimer == 0) {
          neutralExitTimer = millis();
          break;
        }

        if (millis() - neutralExitTimer < 100) {
          break;
        }

        // ------------------------------------
        // Gearbox is now engaged enough that
        // ratio is meaningful.
        // ------------------------------------

        calcGear();

        float ratioError =
          fabs(Ratiocalc - shiftTargetRatio);

        if (ratioError < SHIFT_RATIO_TOLERANCE) {
          if (!targetGearSeen) {
            targetGearSeen = true;
            targetGearTimer = millis();
          }

          if (millis() - targetGearTimer >= GEAR_CONFIRM_TIME) {
            abortShift();
            return;
          }
        } else {
          targetGearSeen = false;
          targetGearTimer = 0;
        }

        // ------------------------------------
        // Failsafe:
        // If we've been out of neutral for
        // 1.5 seconds just assume the driver
        // has selected a gear and stop
        // rev matching.
        // ------------------------------------

        if (millis() - neutralExitTimer >= 1500) {
          abortShift();
          return;
        }

        break;
      }
  }
}


// --------------------------------------------------
// Traction Control
// --------------------------------------------------
void updateTractionControlToggle() {
  bool bothButtons =
    upBtn.stableState && downBtn.stableState;

  bool stationary =
    frontWheelSpeedKmh <= 1.0f && rearWheelSpeedKmh <= 1.0f && motorRPM < 100;

  if (!bothButtons) {
    tcButtonHoldStart = 0;
    tcToggleHandled = false;
    return;
  }

  // Don't treat UP + DOWN as TC control
  // while the car is moving.
  if (!stationary) {
    tcButtonHoldStart = 0;
    tcToggleHandled = false;
    return;
  }

  if (tcButtonHoldStart == 0)
    tcButtonHoldStart = millis();

  unsigned long heldTime =
    millis() - tcButtonHoldStart;

  if (heldTime >= 3000 && !tcToggleHandled) {
    tractionControlEnabled =
      !tractionControlEnabled;

    tcToggleHandled = true;

    startTachConfirmation(
      tractionControlEnabled ? 2 : 1);

    Serial.print("Traction Control: ");

    if (tractionControlEnabled)
      Serial.println("ON");
    else
      Serial.println("OFF");
  }
}



void updateTractionControl()
{
  unsigned long now = millis();

  // ------------------------------------------------------------
  // Calculate loop time
  // ------------------------------------------------------------

  float dt = 0.01f;

  if (tcLastUpdate != 0) {
    dt = (float)(now - tcLastUpdate) / 1000.0f;
  }

  tcLastUpdate = now;

  // Protect against bad / very long timing gaps
  if (dt <= 0.0f || dt > 0.100f)
    dt = 0.01f;


  // ------------------------------------------------------------
  // TC disabled
  // ------------------------------------------------------------

  if (!tractionControlEnabled) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  // ------------------------------------------------------------
  // Only operate with a valid forward gear
  // ------------------------------------------------------------

  if (Gear < 1 || Gear > 5) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  // ------------------------------------------------------------
  // Don't interfere with shifting
  // ------------------------------------------------------------

  if (shiftState != SHIFT_IDLE) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  // ------------------------------------------------------------
  // Throttle fault
  // ------------------------------------------------------------

  if (throttleFault) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  // ------------------------------------------------------------
  // Don't apply TC while braking
  // ------------------------------------------------------------

  if (brakePressed) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  // ------------------------------------------------------------
  // Calculate driver throttle percentage
  // ------------------------------------------------------------

  int throttlePercent =
    ((int)throttle1_scaled - 200) * 100 / (3500 - 200);

  throttlePercent =
    constrain(throttlePercent, 0, 100);


  // ------------------------------------------------------------
  // Don't activate TC at light throttle
  // ------------------------------------------------------------

  if (throttlePercent < TC_MIN_THROTTLE_PERCENT) {

    tcTorquePercent = 100;

    tcIntegral = 0.0f;
    tcPreviousError = 0.0f;

    return;
  }


  tractionControlActive = true;


  // ------------------------------------------------------------
  // Determine actual slip
  // ------------------------------------------------------------

  float actualSlip = 0.0f;
  float targetSlip = TC_TARGET_SLIP_PERCENT;


  // Low speed:
  // use absolute km/h difference because percentage slip
  // becomes meaningless near zero vehicle speed.
  if (frontWheelSpeedKmh < TC_LOW_SPEED_KMH) {

    actualSlip = wheelSlipKmh;

    targetSlip = TC_LOW_SPEED_SLIP_KMH;

  }

  // Normal speed:
  // use percentage slip.
  else {

    actualSlip = wheelSlipPercent;

    targetSlip = TC_TARGET_SLIP_PERCENT;
  }


  // ------------------------------------------------------------
  // PID error
  // ------------------------------------------------------------

  float error =
    actualSlip - targetSlip;


  // ------------------------------------------------------------
  // No meaningful slip
  //
  // Allow the integral to unwind rather than holding
  // a previous torque reduction indefinitely.
  // ------------------------------------------------------------

  if (error <= 0.0f) {

    tcIntegral *= 0.95f;

  }

  else {

    tcIntegral += error * dt;
  }


  // ------------------------------------------------------------
  // Integral anti-windup
  // ------------------------------------------------------------

  tcIntegral =
    constrain(tcIntegral, 0.0f, 100.0f);


  // ------------------------------------------------------------
  // Derivative
  // ------------------------------------------------------------

  float derivative =
    (error - tcPreviousError) / dt;


  tcPreviousError = error;


  // ------------------------------------------------------------
  // PID calculation
  // ------------------------------------------------------------

  uint8_t gear =
    constrain(Gear, 1, 5);


  float correction =
      (TC_KP[gear] * error)
    + (TC_KI[gear] * tcIntegral)
    + (TC_KD[gear] * derivative);


  // ------------------------------------------------------------
  // Only positive correction can reduce torque
  // ------------------------------------------------------------

  if (correction < 0.0f)
    correction = 0.0f;


  // Maximum possible correction
  if (correction > 100.0f)
    correction = 100.0f;


  // ------------------------------------------------------------
  // Requested torque
  // ------------------------------------------------------------

  float requestedTorque =
    100.0f - correction;


  requestedTorque =
    constrain(requestedTorque, 0.0f, 100.0f);


  // ------------------------------------------------------------
  // Final torque slew limiting
  //
  // This prevents the PID itself from suddenly moving torque
  // by a large amount in one control cycle.
  // ------------------------------------------------------------

  float maxDown =
    TC_MAX_TORQUE_DOWN_PER_SEC * dt;

  float maxUp =
    TC_MAX_TORQUE_UP_PER_SEC * dt;


  float currentTorque =
    (float)tcTorquePercent;


  // Torque reduction
  if (requestedTorque < currentTorque) {

    if (requestedTorque <
        currentTorque - maxDown) {

      requestedTorque =
        currentTorque - maxDown;
    }

  }

  // Torque restoration
  else if (requestedTorque > currentTorque) {

    if (requestedTorque >
        currentTorque + maxUp) {

      requestedTorque =
        currentTorque + maxUp;
    }
  }


  // ------------------------------------------------------------
  // Final limits
  // ------------------------------------------------------------

  requestedTorque =
    constrain(requestedTorque, 0.0f, 100.0f);


  tcTorquePercent =
    (uint8_t)(requestedTorque + 0.5f);
}


void startTachConfirmation(uint8_t sweeps) {
  tcTachOverride = true;
  tcTachStartTime = millis();
  tcTachSweepCount = sweeps;
}



void updateRatioCalibration() {
  // --------------------------------------------------
  // Stability calculations
  // --------------------------------------------------

  int motorDelta = abs(motorRPM - lastMotorRPM);
  int transDelta = abs(transmissionSpeed - lastTransmissionSpeed);
  float ratioDelta = fabs(Ratiocalc - lastRatioCalc);

  learnGear = Gear;

  // Track gear stability
  if (learnGear != lastLearnGear) {
    lastLearnGear = learnGear;
    gearStableTimer = millis();
  }

  ratioLearningActive = false;

  // --------------------------------------------------
  // Reject invalid conditions
  // --------------------------------------------------

  if (shiftState != SHIFT_IDLE)
    goto exit;

  if (gearNBtn.stableState)
    goto exit;

  if (learnGear < 1 || learnGear > 5)
    goto exit;

  if (throttle1_scaled < 400)
    goto exit;

  // Gear must be stable for at least 2 seconds
  if (millis() - gearStableTimer < 2000)
    goto exit;

  // Engine / road speed must be almost constant
  if (motorDelta > 50)
    goto exit;

  if (transDelta > 10)
    goto exit;

  // Ratio itself must also be stable
  if (ratioDelta > 0.005f)
    goto exit;


  // --------------------------------------------------
  // Outlier rejection
  // --------------------------------------------------

  float expectedRatio;

  if (ratioCount[learnGear] == 0)
    expectedRatio = learnedRatio[learnGear];
  else
    expectedRatio = ratioAverage[learnGear];

  // Reject anything more than ~2% away
  if (fabs(Ratiocalc - expectedRatio) > 0.03f)
    goto exit;

  // --------------------------------------------------
  // Accept sample
  // --------------------------------------------------

  ratioLearningActive = true;

  ratioCount[learnGear]++;

  if (ratioCount[learnGear] == 1) {
    // First sample
    ratioAverage[learnGear] = Ratiocalc;
  } else {
    // Exponential moving average
    constexpr float alpha = 0.01f;

    ratioAverage[learnGear] +=
      alpha * (Ratiocalc - ratioAverage[learnGear]);
  }

exit:

  lastMotorRPM = motorRPM;
  lastTransmissionSpeed = transmissionSpeed;
  lastRatioCalc = Ratiocalc;
}
// --------------------------------------------------
// Update MIL / Temp / ODO frame
// --------------------------------------------------


int scaleMotorRPMToDash(int rpm)
{
  if (rpm <= 0)
    return 0;

  int dashRPM =
    (int)((float)rpm * 3.85f);

  return constrain(dashRPM, 0, 34650);
}


void updateMIL() {
  send420[0] = engTemp;
  send420[4] = oilPressure ? 1 : 0;
}
// --------------------------------------------------
// RX8 Dash Update (20ms – BIG ENDIAN)
// --------------------------------------------------
int getDashRPM() {
  // -----------------------------------------
  // TC button confirmation
  // -----------------------------------------

  if (tcButtonHoldStart != 0 && !tcToggleHandled) {
    unsigned long heldTime =
      millis() - tcButtonHoldStart;


    if (heldTime >= 3000
    )
      return scaleMotorRPMToDash(3000);

    if (heldTime >= 2000)
      return scaleMotorRPMToDash(2000);

    if (heldTime >= 1000)
      return scaleMotorRPMToDash(1000);
  }
// -----------------------------------------
// Tach sweep confirmation
// -----------------------------------------

if (tcTachOverride)
{
  unsigned long elapsed =
    millis() - tcTachStartTime;

  // Hold 3000 RPM indication before sweep
  if (elapsed < TC_TACH_START_DELAY)
  {
    return scaleMotorRPMToDash(3000);
  }

  elapsed -= TC_TACH_START_DELAY;

  // One complete sweep:
  //
  // 0 RPM       500 ms
  // 9000 RPM    500 ms
  // 0 RPM       500 ms

  const unsigned long sweepTime =
    TC_TACH_LOW_TIME +
    TC_TACH_HIGH_TIME +
    TC_TACH_LOW_TIME;

  unsigned long totalTime =
    (unsigned long)tcTachSweepCount * sweepTime;

  // Finished
  if (elapsed >= totalTime)
  {
    tcTachOverride = false;
    return engineRPM;
  }

  unsigned long phase =
    elapsed % sweepTime;

  // LOW
  if (phase < TC_TACH_LOW_TIME)
  {
    return scaleMotorRPMToDash(0);
  }

  // HIGH
  if (phase <
      (TC_TACH_LOW_TIME + TC_TACH_HIGH_TIME))
  {
    return scaleMotorRPMToDash(9000);
  }

  // LOW
  return scaleMotorRPMToDash(0);
}

  // -----------------------------------------
  // Normal dashboard RPM
  // -----------------------------------------

  return engineRPM;
}


void updatePCM() {
  int dashRPM = getDashRPM();

  int tempVehicleSpeed =
    (vehicleSpeed * 100) + 10000;



  send201[0] = highByte(dashRPM);
  send201[1] = lowByte(dashRPM);
  send201[4] = highByte(tempVehicleSpeed);
  send201[5] = lowByte(tempVehicleSpeed);

  CanMsg msg(0x201, 8, send201);
  CAN.write(msg);
  canTxCount++;
  lastTxID = msg.id;
}


void sendODO() {
  if (ODOus <= 4500000)
    send420[1]++;

  updateMIL();

  CanMsg msg(0x420, 7, send420);
  // CAN.write(msg);
  if (!CAN.write(msg)) {
    Serial.println("CAN 420 FAILED");
  }
  canTxCount++;
  lastTxID = msg.id;
}



void sendFrame(uint16_t id, uint8_t dlc, uint8_t *data)
{
  CanMsg msg(id, dlc, data);

  if (CAN.write(msg)) {
    canTxCount++;
    lastTxID = id;
  }
}

void servicePCMBurst()

{
  unsigned long now = micros();

  switch (pcmState) {
    case PCM_IDLE:
      break;

    case PCM_203:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x203, 7, send203);

      pcmNextFrameTime += 250;
      pcmState = PCM_215;
      break;


    case PCM_215:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x215, 8, send215);

      pcmNextFrameTime += 250;
      pcmState = PCM_231;
      break;


    case PCM_231:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x231, 8, send231);

      pcmNextFrameTime += 250;
      pcmState = PCM_240;
      break;


    case PCM_240:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x240, 8, send240);

      pcmNextFrameTime += 250;
      pcmState = PCM_620;
      break;


    case PCM_620:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x620, 7, send620);

      pcmNextFrameTime += 250;
      pcmState = PCM_630;
      break;


    case PCM_630:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x630, 8, send630);

      pcmNextFrameTime += 250;
      pcmState = PCM_650;
      break;


    case PCM_650:

      if (now < pcmNextFrameTime)
        break;

      sendFrame(0x650, 1, send650);

      pcmNextFrameTime += 250;
      pcmState = PCM_201;
      break;


    case PCM_201:

      if (now < pcmNextFrameTime)
        break;

      updatePCM();

      pcmState = PCM_IDLE;
      break;
  }
}



// --------------------------------------------------
// OpenInverter Command (10ms – LITTLE ENDIAN)
// --------------------------------------------------
void sendOpenInverterCommand() {

  // ------ Brake switch----

  brakePressed = digitalRead(brakeSwitchPin);

  // ----- Read throttle -----

  throttle1_raw = analogRead(throttle1Pin);
  throttle2_raw = analogRead(throttle2Pin);

  /*
    static unsigned long lastThrottleRawDebug = 0;

    if (millis() - lastThrottleRawDebug >= 500)
    {
      lastThrottleRawDebug = millis();

      Serial.print("RAW T1: ");
      Serial.print(throttle1_raw);

      Serial.print("  RAW T2: ");
      Serial.println(throttle2_raw);
    }
  */

  bool throttle1OutOfRange =
    (throttle1_raw < (throttle1Min - THROTTLE_RAW_MARGIN)) || (throttle1_raw > (throttle1Max + THROTTLE_RAW_MARGIN));

  bool throttle2OutOfRange =
    (throttle2_raw < (throttle2Min - THROTTLE_RAW_MARGIN)) || (throttle2_raw > (throttle2Max + THROTTLE_RAW_MARGIN));

  if (throttle1OutOfRange || throttle2OutOfRange) {
    throttleRangeFault = true;
    throttleFault = true;
  }


  // Saturate raw ADC values to calibrated range
  throttle1_raw = constrain(
    throttle1_raw,
    throttle1Min,
    throttle1Max);

  throttle2_raw = constrain(
    throttle2_raw,
    throttle2Min,
    throttle2Max);

  throttle1_scaled =
    map(throttle1_raw,
        throttle1Min,
        throttle1Max,
        200,
        3500);

  throttle2_scaled =
    map(throttle2_raw,
        throttle2Min,
        throttle2Max,
        200,
        3500);

  // Optional inversion
  if (invertThrottle1) {
    throttle1_scaled = 4095 - throttle1_scaled;
  }

  if (invertThrottle2) {
    throttle2_scaled = 4095 - throttle2_scaled;
  }

  // Clamp
  throttle1_scaled = constrain(throttle1_scaled, 0, 4095);
  throttle2_scaled = constrain(throttle2_scaled, 0, 4095);


  // Difference between channels
  throttle_diff =
    abs((int)throttle1_scaled - (int)throttle2_scaled);

  // -----------------------------
  // Fault detection (LATCHING)
  // -----------------------------
  if (throttle_diff > throttleMaxDiff) {
    throttleFault = true;

    Serial.println("Throttle mismatch fault!");
  }

  // -----------------------------
  // Determine idle BEFORE any modification
  // -----------------------------
  bool throttleAtIdle =
    (throttle1_scaled < THROTTLE_IDLE_THRESHOLD) && (throttle2_scaled < THROTTLE_IDLE_THRESHOLD);

  // -----------------------------
  // Fault handling
  // -----------------------------
  if (throttleFault) {
    // Allow fault to clear only if BOTH pedals are at idle for a period
    if (throttleAtIdle && !throttle1OutOfRange && !throttle2OutOfRange) {
      if (millis() - throttleFaultClearTimer > THROTTLE_FAULT_CLEAR_TIME) {
        throttleFault = false;
        throttleRangeFault = false;
      }
    } else {
      // reset timer if not idle
      throttleFaultClearTimer = millis();
    }


  } else {
    // keep timer aligned when no fault present
    throttleFaultClearTimer = millis();
  }

  // Values used in CAN frame
  uint16_t pot = throttle1_scaled;
  uint16_t pot2 = throttle2_scaled;

  // Throttle safety:
  // Keep measured throttle values for diagnostics,
  // but never send torque while a throttle fault is active
  if (throttleFault) {
    pot = 0;
    pot2 = 0;
  }

  // ----- Read regen -----

  regen_raw = analogRead(regenPin);

  regen_scaled = regen_raw * 4;

  if (invertRegen)
    regen_scaled = 4095 - regen_scaled;

  regen_scaled = constrain(regen_scaled, 0, 4095);

  if (regen_scaled < 50)
    regen_scaled = 0;

  regenPreset = regen_scaled >> 5;

  if (regenPreset > 100)
    regenPreset = 100;


  updateShiftLogic();
  updateTractionControl();

  // -----------------------------
  // Update button states
  // -----------------------------



  cruiseTarget = shiftCruiseTarget;
  cruiseTarget = constrain(cruiseTarget, 0, 16383);



  //------CAN I/O--------

  uint8_t canio = 0;

  // Cruise active for shifting
  if (shiftCruiseEnable)
    canio |= (1 << 0);

  // Enable inverter
  //canio |= (1 << 1);

  // Brake pedal
  //if (brakePressed)
  //    canio |= (1 << 2);

  // Always forward
  canio |= (1 << 3);

  // Reverse never used

  // BMS OK
  //canio |= (1 << 5);

  // -----------------------------
  // Unload drivetrain during shift
  // -----------------------------
  // -----------------------------
  // Shift torque ramp
  // Runs every 10 ms
  // -----------------------------

  if (shiftTorqueCut) {
    // Ramp torque down
    if (shiftTorquePercent > SHIFT_TORQUE_RAMP_DOWN)
      shiftTorquePercent -= SHIFT_TORQUE_RAMP_DOWN;
    else
      shiftTorquePercent = 0;

    // No regen during the shift
    regenPreset = 0;
  } else {
    // Ramp driver torque back in
    if (shiftTorquePercent < (100 - SHIFT_TORQUE_RAMP_UP))
      shiftTorquePercent += SHIFT_TORQUE_RAMP_UP;
    else
      shiftTorquePercent = 100;
  }
  // Apply torque percentage to both throttle channels
  // Ramp between calibrated idle (200) and driver request
  // Do not modify the zero command used for a throttle fault

  // -----------------------------------------
  // Combine shift torque limit and TC limit
  // -----------------------------------------

  uint8_t finalTorquePercent =
    min(shiftTorquePercent, tcTorquePercent);

  // -----------------------------------------
  // Apply torque percentage to both throttle
  // channels
  // -----------------------------------------

  if (!throttleFault) {
    pot =
      200 + (((uint32_t)(pot - 200) * finalTorquePercent) / 100);

    pot2 =
      200 + (((uint32_t)(pot2 - 200) * finalTorquePercent) / 100);
  }


  ///// ----- Rolling counters -----
  canrun1 = (canrun1 + 1) & 0x03;
  canrun2 = (canrun2 + 1) & 0x03;

  // ----- Pack Little Endian frame -----
  uint64_t msg64 = 0;
  msg64 |= (uint64_t)pot;
  msg64 |= (uint64_t)pot2 << 12;
  msg64 |= (uint64_t)(canio & 0x3F) << 24;
  msg64 |= (uint64_t)(canrun1 & 0x03) << 30;
  msg64 |= (uint64_t)(cruiseTarget & 0x3FFF) << 32;
  msg64 |= (uint64_t)(canrun2 & 0x03) << 46;
  msg64 |= (uint64_t)(regenPreset & 0xFF) << 48;

  uint8_t payload[8];

  for (int i = 0; i < 8; i++) {
    payload[i] = (msg64 >> (8 * i)) & 0xFF;
  }

  // CRC byte cleared before calculation
  payload[7] = 0;


  uint32_t crcWords[2];

  crcWords[0] =
    ((uint32_t)payload[0]) | ((uint32_t)payload[1] << 8) | ((uint32_t)payload[2] << 16) | ((uint32_t)payload[3] << 24);

  crcWords[1] =
    ((uint32_t)payload[4]) | ((uint32_t)payload[5] << 8) | ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);


  if (canDumpEnabled) {
    Serial.print("OI DATA ");

    for (int i = 0; i < 8; i++) {
      if (payload[i] < 16)
        Serial.print("0");

      Serial.print(payload[i], HEX);
      Serial.print(" ");
    }

    Serial.println();
  }

  payload[7] = computeOpenInverterCRC(crcWords);
  if (canDumpEnabled) {
    Serial.print("CRC = 0x");

    if (payload[7] < 16)
      Serial.print("0");

    Serial.println(payload[7], HEX);
  }
  CanMsg msg(INVERTER_CMD_ID, 8, payload);
  CAN.write(msg);

  canTxCount++;
  lastTxID = msg.id;
}

// --------------------------------------------------

void setup() {
  Serial.begin(500000);

  serialConsoleInit();
  settingsLoad();

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(UP_pin, INPUT);
  pinMode(DOWN_pin, INPUT);
  pinMode(GearN_pin, INPUT);
  pinMode(Oil_pump, OUTPUT);
  pinMode(brakeSwitchPin, INPUT);


  if (!CAN.begin(CanBitRate::BR_500k)) {
    Serial.println("CAN init failed");
    while (1)
      ;
  }
  Serial.println("CAN started");
  lastInverterUpdate = millis();
  lastPCMUpdate = millis();
  lastDebugUpdate = millis();
  lastLog = millis();
  ODORefreshTime = micros();
  updateMIL();
  engineRPM = 2000;  // temporary RPM to wake EPS
  motorRPM = 0;
  vehicleSpeed = 0;
  Serial.println("RX8 VCU – Nano R4 Ready");
}

// --------------------------------------------------

void loop() {
  servicePCMBurst();

  updateDebounce(upBtn);
  updateDebounce(downBtn);
  updateDebounce(gearNBtn);
  upButton = upBtn.stableState;
  downButton = downBtn.stableState;
  neutralButton = gearNBtn.stableState;

  updateTractionControlToggle();

  // -----------------------------
  // Shift trigger
  // -----------------------------

  bool upPressed =
    upBtn.stableState;

  bool downPressed =
    downBtn.stableState;

 bool combinedTCButtonHold =
  upBtn.stableState &&
  downBtn.stableState;

bool upEdge =
  upPressed &&
  !lastUpState &&
  shiftState == SHIFT_IDLE &&
  !combinedTCButtonHold;

bool downEdge =
  downPressed &&
  !lastDownState &&
  shiftState == SHIFT_IDLE &&
  !combinedTCButtonHold;

  if (upEdge || downEdge) {
    // Get the freshest possible gear estimate
    calcGear();

    bool validUpshift =
      upEdge && Gear >= 1 && Gear < 5;

    bool validDownshift =
      downEdge && Gear > 1 && Gear <= 5;

    if (validUpshift || validDownshift) {
      // Remember the gear we're leaving
      latchedGear = Gear;

      // Remember direction
      if (validUpshift)
        shiftDirection = SHIFT_UP;
      else
        shiftDirection = SHIFT_DOWN;

      // Calculate target once
      if (shiftDirection == SHIFT_UP)
        shiftTargetGear = latchedGear + 1;
      else
        shiftTargetGear = latchedGear - 1;

      shiftTargetGear = constrain(shiftTargetGear, 1, 5);
      shiftTargetRatio = learnedRatio[shiftTargetGear];
      targetGearSeen = false;
      targetGearTimer = 0;

      gearEngagedSeen = false;
      gearEngagedTimer = 0;
      neutralExitTimer = 0;

      shiftStartTime = millis();

      shiftState = SHIFT_TORQUE_CUT;
    }
  }

  lastUpState = upPressed;
  lastDownState = downPressed;

  //-----------------------------------------------
  // Task scheduler
  //-----------------------------------------------

  unsigned long now = millis();

  unsigned long nowMicros = micros();

  while (now - lastInverterUpdate >= 10) {
    lastInverterUpdate += 10;
    sendOpenInverterCommand();
  }
  while (now - lastLog >= 20) {
    lastLog += 20;

    if (loggingEnabled)
      serialLogOutput();
  }


  while (now - lastPCMUpdate >= 75
  ) {
    lastPCMUpdate += 75;

    if (pcmState == PCM_IDLE) {
      pcmState = PCM_203;
      pcmNextFrameTime = micros();
    } else {
      // We overran the previous burst
      pcmOverruns++;
    }
  }


  while (now - lastDebugUpdate >= 1000) {
    lastDebugUpdate += 1000;

    if (debugEnabled) {
      debugOutput();
    }
  }


  while (nowMicros - ODORefreshTime >= ODOus) {
    ODORefreshTime += ODOus;
    sendODO();
  }




  //---------------------------------------------------
  //CAN Recieve
  //---------------------------------------------------

  while (CAN.available()) {
    CanMsg msg = CAN.read();
    canRxCount++;
    lastRxID = msg.id;
    if (canDumpEnabled) {
      Serial.print(millis());

      Serial.print(" RX 0x");

      Serial.print(msg.id, HEX);

      Serial.print(" [");

      Serial.print(msg.data_length);

      Serial.print("] ");

      for (int i = 0; i < msg.data_length; i++) {
        if (msg.data[i] < 16)
          Serial.print("0");

        Serial.print(msg.data[i], HEX);

        Serial.print(" ");
      }

      Serial.println();
    }

    // Motor RPM from inverter
    if (msg.id == 10 && msg.data_length >= 2) {
      inverterSeen = true;
      lastInverterMessage = millis();

      int rawRpm =
        (msg.data[1] << 8) | msg.data[0];

      if (rawRpm <= 10000) {
        motorRPM = rawRpm;
        engineRPM = scaleMotorRPMToDash(rawRpm);
      } else {
        motorRPM = 9000;
        engineRPM = 9000;
      }
    }


    if (msg.id == 15 && msg.data_length >= 1) {
      engTemp = map(msg.data[0], 0, 254, 88, 230);

      inverterSeen = true;
      lastInverterMessage = millis();
    }


    // Wheel speed from ABS

    if (msg.id == 0x4B0 && msg.data_length >= 8)

    {


      int frontLeft =
        (msg.data[0] << 8) | msg.data[1];

      int frontRight =
        (msg.data[2] << 8) | msg.data[3];

      int rearLeft =
        (msg.data[4] << 8) | msg.data[5];

      int rearRight =
        (msg.data[6] << 8) | msg.data[7];

      // -----------------------------------------
      // Wheel speeds
      // -----------------------------------------

      frontWheelSpeedKmh =
        ((((float)frontLeft + (float)frontRight) / 2.0f)
         - 10000.0f)
        / 100.0f;

      rearWheelSpeedKmh =
        ((((float)rearLeft + (float)rearRight) / 2.0f)
         - 10000.0f)
        / 100.0f;

      // Existing vehicle speed values
      vehicleSpeed =
        (int)frontWheelSpeedKmh;

      transmissionSpeed =
        (int)rearWheelSpeedKmh;

      // -----------------------------------------
      // Wheel slip
      // -----------------------------------------

      wheelSlipKmh =
        rearWheelSpeedKmh - frontWheelSpeedKmh;

      if (wheelSlipKmh < 0.0f)
        wheelSlipKmh = 0.0f;

      if (frontWheelSpeedKmh > 0.5f) {
        wheelSlipPercent =
          (wheelSlipKmh / frontWheelSpeedKmh) * 100.0f;
      } else {
        wheelSlipPercent = 0.0f;
      }

      // -----------------------------------------
      // Existing functions
      // -----------------------------------------

      ODOus =
        calcMicrosecODO(vehicleSpeed * 100);

      calcGear();

      if (ratioLearningEnabled) {
        updateRatioCalibration();
      }
    }

    // Inverter comms timeout
    if (millis() - lastInverterMessage > 5000) {
      inverterSeen = false;

      motorRPM = 0;
      engineRPM = 5000;
    }
    if (Serial.available()) {
      serialConsoleTask();
    }

    //-------------------------------------------------------
    //Output fet control
    //-------------------------------------------------------
    // Oil Pump Control
    if (inverterSeen && motorRPM > 200) {
      lastEngineRun = millis();
    }

    if (millis() - lastEngineRun < 3000) {
      digitalWrite(Oil_pump, HIGH);
    } else {
      digitalWrite(Oil_pump, LOW);
    }

    // ---------------------------
    // Heartbeat LED
    // ---------------------------
    if (millis() - lastHeartbeat > 500) {
      lastHeartbeat = millis();

      heartbeatState = !heartbeatState;

      digitalWrite(LED_BUILTIN, heartbeatState);
    }
    if (millis() - lastCanDebugTime >= 1000) {
      lastCanDebugTime += 1000;

      txPerSecond = canTxCount - lastTxSnapshot;
      rxPerSecond = canRxCount - lastRxSnapshot;

      lastTxSnapshot = canTxCount;
      lastRxSnapshot = canRxCount;
    }
  }
}