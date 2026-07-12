

//#include <Arduino_CAN.h>
#include <CanUtil.h>
#include <R7FA4M1_CAN.h>
#include <R7FA6M5_CAN.h>
#include <SyncCanMsgRingbuffer.h>

#include <Arduino.h>
#include <Arduino_CAN.h>
#include "serial_console.h"
#include "vcu_settings.h"

struct DebounceInput {
  uint8_t pin;
  bool stableState;
  bool lastReading;
  unsigned long lastChangeTime;
};

#define INVERTER_CMD_ID 0x3F
#define DEBOUNCE_TIME 15

unsigned long shiftStartTime = 0;


#define SHIFT_MAX_TIME     3000  // ms (safety timeout)
#define GEAR_CONFIRM_TIME 50
unsigned long targetGearTimer = 0;


// --------------------------------------------------
// Timing
// --------------------------------------------------

unsigned long lastInverterUpdate = 0;
unsigned long lastPCMUpdate = 0;
unsigned long lastDebugUpdate = 0;
unsigned long lastEngineRun = 0;
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

//led blink
unsigned long lastHeartbeat = 0;
bool heartbeatState = false;


// ---------- CAN Diagnostics ----------
uint32_t canTxCount = 0;
uint32_t canRxCount = 0;

uint32_t lastTxID = 0;
uint32_t lastRxID = 0;

uint32_t lastCanDebugTime = 0;

uint32_t txPerSecond = 0;
uint32_t rxPerSecond = 0;

uint32_t lastTxSnapshot = 0;
uint32_t lastRxSnapshot = 0;

bool canDumpEnabled = false;
bool clutchPressed = false;
bool ignitionOn = false;
bool reverseSelected = false;

uint32_t pcmOverruns = 0;

//--------------------------------------------------
// Odemeter
//---------------------------------------------------
long ODOus = 4500000;
unsigned long ODORefreshTime = 0;

// --------------------------------------------------
// Pins
// --------------------------------------------------


//inputs digital
const int UP_pin = 7;
const int DOWN_pin = 8;
const int GearN_pin = 9;
const int brakeSwitchPin = 12;


//inputs digital 12v isolated
//const int optoInput1 = 2;
//const int optoInput2 = 3;

//inputs anologe
const int throttle1Pin = A4;
const int throttle2Pin = A3;
const int regenPin = A2;
//const int spare analog = A1;
//const int spare analog = A0;

//outputs
const int Oil_pump = 13;

//const int out2 = 10;
//const int out3 = 11;
//const int out4 = 6;


// --------------------------------------------------
// Debounce Structure
// --------------------------------------------------


DebounceInput upBtn    = {UP_pin, LOW, LOW, 0};
DebounceInput downBtn  = {DOWN_pin, LOW, LOW, 0};
DebounceInput gearNBtn = {GearN_pin, LOW, LOW, 0};

void updateDebounce(DebounceInput &input)
{
  bool reading = digitalRead(input.pin);

  if (reading != input.lastReading)
    input.lastChangeTime = millis();

  if ((millis() - input.lastChangeTime) > DEBOUNCE_TIME)
    input.stableState = reading;

  input.lastReading = reading;
}


// --------------------------------------------------
// Inverter State
// --------------------------------------------------
uint8_t canrun1 = 0;
uint8_t canrun2 = 0;
uint16_t cruiseTarget = 0;
uint8_t regenPreset = 0;
bool inverterSeen = false;
unsigned long lastInverterMessage = 0;


// --------------------------------------------------
// RX8 Dash / PCM Emulation Frames
// --------------------------------------------------
enum PCMBurstState
{
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

////CAN frame packing helper//////

inline void sendFrame(uint16_t id, uint8_t dlc, uint8_t *data)
{
    CanMsg msg(id, dlc, data);

    if (CAN.write(msg))
    {
        canTxCount++;
        lastTxID = id;
    }
    else
    {
        Serial.print("TX FAIL 0x");
        Serial.println(id, HEX);
    }
}

unsigned long pcmNextFrameTime = 0;


uint8_t send420[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t send203[7] = {19, 19, 19, 19, 175, 3, 19};
uint8_t send215[8] = {2, 45, 2, 45, 2, 42, 6, 129};
uint8_t send231[8] = {15, 0, 255, 255, 0, 0, 0, 0};
uint8_t send240[8] = {4, 0, 40, 0, 2, 55, 6, 129};
uint8_t send620[7] = {0, 0, 0, 0, 0, 0, 4};
uint8_t send630[8] = {8, 0, 0, 0, 0, 0, 106, 106};
uint8_t send650[1] = {0};
uint8_t send201[8] = {0, 0, 255, 255, 0, 0, 0, 255};


int engTemp = 0;
bool oilPressure = true;
int engineRPM = 0;      // scaled for rx8 dash
int motorRPM = 0;      // Actual inverter/motor RPM
int vehicleSpeed = 0;
int transmissionSpeed = 0;


// --------------------------------------------------
// Gear Logic
// --------------------------------------------------


enum ShiftState
{
    SHIFT_IDLE,
    SHIFT_TORQUE_CUT,
    SHIFT_REV_MATCH,
   
};

enum ShiftDirection
{
    SHIFT_NONE,
    SHIFT_UP,
    SHIFT_DOWN
};

float shiftTargetRatio = 0.0;
uint16_t shiftBaseTarget = 0;
uint16_t shiftCruiseTarget = 0;
bool targetGearSeen = false;
bool shiftTorqueCut = false;
bool shiftCruiseEnable = false;
// Shift torque ramp
uint8_t shiftTorquePercent = 100;

// Percentage change per 10 ms inverter command
const uint8_t SHIFT_TORQUE_RAMP_DOWN = 20;
const uint8_t SHIFT_TORQUE_RAMP_UP   = 10;




ShiftState shiftState = SHIFT_IDLE;
ShiftDirection shiftDirection = SHIFT_NONE;
int Gear = 0;
int latchedGear = 0;
int shiftTargetGear = 0;
float Ratioact = 0;
float Ratiocalc = 0;
float wheelRPM = 0;
float gearboxOutputRPM = 0;
bool lastUpState = false;
bool lastDownState = false;

float upshiftGain = 1.000f;
float downshiftGain = 1.000f;

float gearRatio[6] = {
  0.0,
  3.483,   // 1st
  2.015,   // 2nd
  1.391,   // 3rd
  1.000,   // 4th
  0.806    // 5th
};
// Drivetrain constants
const float TYRE_CIRCUMFERENCE_M = 2.082f;  // 225/40 R19
const float FINAL_DRIVE_RATIO = 4.3f;


#define GEAR_HYSTERESIS 0.15

// --------------------------------------------------
// Throttle / Regen Variables
// --------------------------------------------------


unsigned long throttleFaultClearTimer = 0;

#define THROTTLE_IDLE_THRESHOLD 100
#define THROTTLE_FAULT_CLEAR_TIME 1000   // ms
#define THROTTLE_RAW_MARGIN 40
bool throttleFault = false;
bool throttleRangeFault = false;
uint16_t throttleMaxDiff = 300;

bool invertThrottle1 = false;
bool invertThrottle2 = false;

uint16_t throttle1Min = 323;
uint16_t throttle1Max = 794;

uint16_t throttle2Min = 212;
uint16_t throttle2Max = 683;

uint16_t throttle1_raw = 0;
uint16_t throttle2_raw = 0;

uint16_t throttle1_scaled = 0;
uint16_t throttle2_scaled = 0;

uint16_t throttle_diff = 0;

bool debugEnabled = false;
bool invertRegen = false;
uint16_t regen_raw = 0;
uint16_t regen_scaled = 0;

bool brakePressed = false;

bool upButton = false;
bool downButton = false;
bool neutralButton = false;





//----- Open inverter CRC -----
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
void calcGear()
{
  if ((motorRPM < 50) || (transmissionSpeed < 5))
    return;

  // Convert rear road speed (km/h) to wheel RPM
  wheelRPM =
    ((float)transmissionSpeed * 1000.0f) /
    (TYRE_CIRCUMFERENCE_M * 60.0f);

  // Convert wheel RPM to gearbox output shaft RPM
  gearboxOutputRPM =
    wheelRPM * FINAL_DRIVE_RATIO;

  // Calculate actual gearbox ratio
  Ratiocalc =
    (float)motorRPM / gearboxOutputRPM;

  // Find closest gear
  int detectedGear = 0;
  float smallestError = 999.0;

  for (int i = 1; i <= 5; i++)
  {
    float error = abs(Ratiocalc - gearRatio[i]);

    if (error < smallestError)
    {
      smallestError = error;
      detectedGear = i;
    }
  }

  // Apply hysteresis
  if (Gear == 0)
  {
    Gear = detectedGear;
  }
  else
  {
    float currentError = abs(Ratiocalc - gearRatio[Gear]);
    float newError = abs(Ratiocalc - gearRatio[detectedGear]);

    if (newError + GEAR_HYSTERESIS < currentError)
    {
      Gear = detectedGear;
    }
  }

  // Update active ratio for debug/other use
  Ratioact = gearRatio[Gear];
}

void abortShift()
{
    shiftDirection = SHIFT_NONE;
    shiftTargetGear = 0;
    shiftTargetRatio = 0;
    shiftBaseTarget = 0;
    shiftCruiseTarget = 0;

    shiftTorqueCut = false;
    shiftCruiseEnable = false;

    shiftState = SHIFT_IDLE;
    targetGearSeen = false;
}


// 
void updateShiftLogic()
{
    shiftTorqueCut = false;
    shiftCruiseEnable = false;
    shiftCruiseTarget = 0;

     // -----------------------------
    // Overall shift safety timeout
    // -----------------------------
    if (shiftState != SHIFT_IDLE &&
        millis() - shiftStartTime >= SHIFT_MAX_TIME)
    {
        abortShift();
        return;
    }

    switch (shiftState)
    {
        case SHIFT_IDLE:
            break;

        case SHIFT_TORQUE_CUT:

    shiftTorqueCut = true;

    if (gearNBtn.stableState &&
    shiftTorquePercent == 0)
{
    if (shiftDirection == SHIFT_UP)
        shiftTargetGear = latchedGear + 1;

    else if (shiftDirection == SHIFT_DOWN)
        shiftTargetGear = latchedGear - 1;

    shiftTargetGear = constrain(shiftTargetGear, 1, 5);

    shiftTargetRatio = gearRatio[shiftTargetGear];

    shiftState = SHIFT_REV_MATCH;
}

    break;

       case SHIFT_REV_MATCH:
{
    shiftTorqueCut = true;
    shiftCruiseEnable = true;

    wheelRPM =
      ((float)transmissionSpeed * 1000.0f) /
      (TYRE_CIRCUMFERENCE_M * 60.0f);

    gearboxOutputRPM =
      wheelRPM * FINAL_DRIVE_RATIO;

    shiftBaseTarget =
      shiftTargetRatio * gearboxOutputRPM;

    float gain =
        (shiftDirection == SHIFT_UP)
            ? upshiftGain
            : downshiftGain;

    shiftCruiseTarget =
        shiftBaseTarget * gain;

    // Continuously monitor the gearbox
    calcGear();

   
    
  if (!gearNBtn.stableState &&
    Gear == shiftTargetGear)
{
    // First time we've seen the correct gear
    if (!targetGearSeen)
    {
        targetGearSeen = true;
        targetGearTimer = millis();
    }

    // Gear has remained engaged long enough
    if (millis() - targetGearTimer >= GEAR_CONFIRM_TIME)
    {
        abortShift();
    }
}
else
{
    // Either back in neutral or wrong gear
    targetGearSeen = false;
    targetGearTimer = 0;
}

    break;
}

     
    }
}



// --------------------------------------------------
// Update MIL / Temp / ODO frame
// --------------------------------------------------



void updateMIL()
{
  send420[0] = engTemp;
  send420[4] = oilPressure ? 1 : 0;
}
// --------------------------------------------------
// RX8 Dash Update (20ms – BIG ENDIAN)
// --------------------------------------------------
void updatePCM()
{
  int tempVehicleSpeed =
    (vehicleSpeed * 100) + 10000;



 send201[0] = highByte(engineRPM);
  send201[1] = lowByte(engineRPM);
  send201[4] = highByte(tempVehicleSpeed);
  send201[5] = lowByte(tempVehicleSpeed);

  CanMsg msg(0x201, 8, send201);
CAN.write(msg);
canTxCount++;
lastTxID = msg.id;
}


void sendODO()
{
  if (ODOus <= 4500000)
    send420[1]++;

  updateMIL();

  CanMsg msg(0x420, 7, send420);
 // CAN.write(msg);
 if (!CAN.write(msg))
{
    Serial.println("CAN 420 FAILED");
}
  canTxCount++;
lastTxID = msg.id;
}

void servicePCMBurst()

{
    unsigned long now = micros();

    switch (pcmState)
    {
        case PCM_IDLE:
            break;

        case PCM_203:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x203, 7, send203);

            pcmNextFrameTime += 500;
            pcmState = PCM_215;
            break;


        case PCM_215:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x215, 8, send215);

            pcmNextFrameTime += 500;
            pcmState = PCM_231;
            break;


        case PCM_231:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x231, 8, send231);

            pcmNextFrameTime += 500;
            pcmState = PCM_240;
            break;


        case PCM_240:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x240, 8, send240);

            pcmNextFrameTime += 500;
            pcmState = PCM_620;
            break;


        case PCM_620:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x620, 7, send620);

            pcmNextFrameTime += 500;
            pcmState = PCM_630;
            break;


        case PCM_630:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x630, 8, send630);

            pcmNextFrameTime += 500;
            pcmState = PCM_650;
            break;


        case PCM_650:

            if (now < pcmNextFrameTime)
                break;

            sendFrame(0x650, 1, send650);

            pcmNextFrameTime += 500;
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
void sendOpenInverterCommand()
{

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
    (throttle1_raw < (throttle1Min - THROTTLE_RAW_MARGIN)) ||
    (throttle1_raw > (throttle1Max + THROTTLE_RAW_MARGIN));

bool throttle2OutOfRange =
    (throttle2_raw < (throttle2Min - THROTTLE_RAW_MARGIN)) ||
    (throttle2_raw > (throttle2Max + THROTTLE_RAW_MARGIN));

if (throttle1OutOfRange || throttle2OutOfRange)
{
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
if (invertThrottle1)
{
    throttle1_scaled = 4095 - throttle1_scaled;
}

if (invertThrottle2)
{
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
if (throttle_diff > throttleMaxDiff)
{
    throttleFault = true;

    Serial.println("Throttle mismatch fault!");
}

// -----------------------------
// Determine idle BEFORE any modification
// -----------------------------
bool throttleAtIdle =
  (throttle1_scaled < THROTTLE_IDLE_THRESHOLD) &&
  (throttle2_scaled < THROTTLE_IDLE_THRESHOLD);

// -----------------------------
// Fault handling
// -----------------------------
if (throttleFault)
{
    // Allow fault to clear only if BOTH pedals are at idle for a period
   if (throttleAtIdle &&
    !throttle1OutOfRange &&
    !throttle2OutOfRange)
    {
        if (millis() - throttleFaultClearTimer > THROTTLE_FAULT_CLEAR_TIME)
        {
            throttleFault = false;
            throttleRangeFault = false;
        }
    }
    else
    {
        // reset timer if not idle
        throttleFaultClearTimer = millis();
    }

    
}
else
{
    // keep timer aligned when no fault present
    throttleFaultClearTimer = millis();
}

// Values used in CAN frame
uint16_t pot = throttle1_scaled;
uint16_t pot2 = throttle2_scaled;

// Throttle safety:
// Keep measured throttle values for diagnostics,
// but never send torque while a throttle fault is active
if (throttleFault)
{
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


// Temporary fixed maximum regen setting
//regenPreset = 100;


  // -----------------------------
// Update gear estimate when not shifting
// -----------------------------
if (shiftState == SHIFT_IDLE)
{
  calcGear();
}

updateShiftLogic();


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

if (shiftTorqueCut)
{
    // Ramp torque down
    if (shiftTorquePercent > SHIFT_TORQUE_RAMP_DOWN)
        shiftTorquePercent -= SHIFT_TORQUE_RAMP_DOWN;
    else
        shiftTorquePercent = 0;

    // No regen during the shift
    regenPreset = 0;
}
else
{
    // Ramp driver torque back in
    if (shiftTorquePercent < (100 - SHIFT_TORQUE_RAMP_UP))
        shiftTorquePercent += SHIFT_TORQUE_RAMP_UP;
    else
        shiftTorquePercent = 100;
}
// Apply torque percentage to both throttle channels
// Ramp between calibrated idle (200) and driver request
// Do not modify the zero command used for a throttle fault

if (!throttleFault)
{
    pot =
        200 +
        (((uint32_t)(pot - 200) * shiftTorquePercent) / 100);

    pot2 =
        200 +
        (((uint32_t)(pot2 - 200) * shiftTorquePercent) / 100);
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

for (int i = 0; i < 8; i++)
{
    payload[i] = (msg64 >> (8 * i)) & 0xFF;
}

// CRC byte cleared before calculation
payload[7] = 0;


uint32_t crcWords[2];

crcWords[0] =
    ((uint32_t)payload[0]) |
    ((uint32_t)payload[1] << 8) |
    ((uint32_t)payload[2] << 16) |
    ((uint32_t)payload[3] << 24);

crcWords[1] =
    ((uint32_t)payload[4]) |
    ((uint32_t)payload[5] << 8) |
    ((uint32_t)payload[6] << 16) |
    ((uint32_t)payload[7] << 24);


if (canDumpEnabled)
{
    Serial.print("OI DATA ");

    for (int i = 0; i < 8; i++)
    {
        if (payload[i] < 16)
            Serial.print("0");

        Serial.print(payload[i], HEX);
        Serial.print(" ");
    }

    Serial.println();
}

payload[7] = computeOpenInverterCRC(crcWords);
if (canDumpEnabled)
{
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

void setup()
{
  Serial.begin(115200);

  serialConsoleInit();
  settingsLoad();
  
pinMode(LED_BUILTIN, OUTPUT);
  pinMode(UP_pin, INPUT);
  pinMode(DOWN_pin, INPUT);
  pinMode(GearN_pin, INPUT);
  pinMode(Oil_pump, OUTPUT);
  pinMode(brakeSwitchPin, INPUT);


  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN init failed");
    while (1);
   
  }
 Serial.println("CAN started");
 lastInverterUpdate = millis();
lastPCMUpdate      = millis();
lastDebugUpdate    = millis();
ODORefreshTime     = micros();
  updateMIL();
  engineRPM = 2000;   // temporary RPM to wake EPS
  motorRPM = 0;
  vehicleSpeed = 0; 
  Serial.println("RX8 VCU – Nano R4 Ready");
}

// --------------------------------------------------

void loop()
{
servicePCMBurst();
 
  updateDebounce(upBtn);
  updateDebounce(downBtn);
  updateDebounce(gearNBtn);
  upButton = upBtn.stableState;
downButton = downBtn.stableState;
neutralButton = gearNBtn.stableState;

// -----------------------------
// Shift trigger
// -----------------------------

bool upPressed =
    upBtn.stableState;

bool downPressed =
    downBtn.stableState;

bool upEdge =
    upPressed &&
    !lastUpState &&
    shiftState == SHIFT_IDLE;

bool downEdge =
    downPressed &&
    !lastDownState &&
    shiftState == SHIFT_IDLE;

   if (upEdge || downEdge)
{
    // Get the freshest possible gear estimate
    calcGear();

    bool validUpshift =
        upEdge &&
        Gear >= 1 &&
        Gear < 5;

    bool validDownshift =
        downEdge &&
        Gear > 1 &&
        Gear <= 5;

    if (validUpshift || validDownshift)
    {
        // Remember the gear we are shifting from
        latchedGear = Gear;
        targetGearSeen = false;
        targetGearTimer = 0;

        // Record when the shift started
        shiftStartTime = millis();

        // Begin by cutting torque
        shiftState = SHIFT_TORQUE_CUT;

        // Remember the requested direction
        if (validUpshift)
            shiftDirection = SHIFT_UP;

        if (validDownshift)
            shiftDirection = SHIFT_DOWN;
    }
}

lastUpState = upPressed;
lastDownState = downPressed;

  //-----------------------------------------------
  // Task scheduler
  //-----------------------------------------------

  unsigned long now = millis();

  unsigned long nowMicros = micros();

   while (now - lastInverterUpdate >= 10)
  {
    lastInverterUpdate += 10;
    sendOpenInverterCommand();
  }

  
while (now - lastPCMUpdate >= 75)
{
    lastPCMUpdate += 75;

    if (pcmState == PCM_IDLE)
    {
        pcmState = PCM_203;
        pcmNextFrameTime = micros();
    }
    else
    {
        // We overran the previous burst
        pcmOverruns++;
    }
}

  
while (now - lastDebugUpdate >= 1000)
{
    lastDebugUpdate += 1000;

    if (debugEnabled)
    {
        debugOutput();
    }
}


  while (nowMicros - ODORefreshTime >= ODOus)
  {
    ODORefreshTime += ODOus;
    sendODO();
  }



  
  //---------------------------------------------------
  //CAN Recieve
  //---------------------------------------------------

  while (CAN.available())
  {
    CanMsg msg = CAN.read();
   canRxCount++;
lastRxID = msg.id;
if (canDumpEnabled)
{
    Serial.print(millis());

    Serial.print(" RX 0x");

    Serial.print(msg.id, HEX);

    Serial.print(" [");

    Serial.print(msg.data_length);

    Serial.print("] ");

    for (int i = 0; i < msg.data_length; i++)
    {
        if (msg.data[i] < 16)
            Serial.print("0");

        Serial.print(msg.data[i], HEX);

        Serial.print(" ");
    }

    Serial.println();
}

    // Motor RPM from inverter
if (msg.id == 10 && msg.data_length >= 2)
{
    inverterSeen = true;
    lastInverterMessage = millis();

    int rawRpm =
        (msg.data[1] << 8) | msg.data[0];

    if (rawRpm <= 10000)
    {
        motorRPM = rawRpm;
        engineRPM = (int)((float)rawRpm * 3.85f);
    }
    else
    {
        motorRPM = 10000;
        engineRPM = 9000;
    }
}


if (msg.id == 15 && msg.data_length >= 1)
{
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

    // Dash speed
    vehicleSpeed =
        (((frontLeft + frontRight) / 2) - 10000) / 100;

    // Transmission speed
    transmissionSpeed =
        (((rearLeft + rearRight) / 2) - 10000) / 100;

    ODOus =
        calcMicrosecODO(vehicleSpeed * 100);
}
  }

// Inverter comms timeout
if (millis() - lastInverterMessage > 5000)
{
    inverterSeen = false;

    motorRPM = 0;
    engineRPM = 5000;
}
  if (Serial.available())
  {
    serialConsoleTask();
  }

  //-------------------------------------------------------
  //Output fet control
  //-------------------------------------------------------
  // Oil Pump Control
if (inverterSeen && motorRPM > 200)
{
    lastEngineRun = millis();
}

if (millis() - lastEngineRun < 3000)
{
    digitalWrite(Oil_pump, HIGH);
}
else
{
    digitalWrite(Oil_pump, LOW);
}

// ---------------------------
// Heartbeat LED
// ---------------------------
if (millis() - lastHeartbeat > 500)
{
    lastHeartbeat = millis();

    heartbeatState = !heartbeatState;

    digitalWrite(LED_BUILTIN, heartbeatState);
}
if (millis() - lastCanDebugTime >= 1000)
{
    lastCanDebugTime += 1000;

    txPerSecond = canTxCount - lastTxSnapshot;
    rxPerSecond = canRxCount - lastRxSnapshot;

    lastTxSnapshot = canTxCount;
    lastRxSnapshot = canRxCount;
}

}
