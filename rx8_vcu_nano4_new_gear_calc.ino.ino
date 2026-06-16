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

#define INVERTER_CMD_ID 0x300
#define DEBOUNCE_TIME 15


unsigned long shiftStartTime = 0;

#define UPSHIFT_MIN_TIME   120
#define DOWNSHIFT_MIN_TIME 200
#define SHIFT_MAX_TIME     1000  // ms (safety timeout)
#define NEUTRAL_EXIT_DELAY 200  // ms


// --------------------------------------------------
// Timing
// --------------------------------------------------
unsigned long lastDashUpdate = 0;
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
//const int spareInput1 = 12;


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
// RX8 Dash Frame
// --------------------------------------------------
bool rpmStartupPulse = true;
// --------------------------------------------------
// RX8 Dash / PCM Emulation Frames
// --------------------------------------------------

uint8_t send420[7] = {0, 0, 0, 0, 0, 0, 0};
uint8_t send203[7] = {19, 19, 19, 19, 175, 3, 19};
uint8_t send215[8] = {2, 45, 2, 45, 2, 42, 6, 129};
uint8_t send231[8] = {15, 0, 255, 255, 0, 0, 0, 0};
uint8_t send240[8] = {4, 0, 40, 0, 2, 55, 6, 129};
uint8_t send620[7] = {0, 0, 0, 0, 0, 0, 4};
uint8_t send630[8] = {8, 0, 0, 0, 0, 0, 106, 106};
uint8_t send650[1] = {0};
uint8_t send201[8] = {0, 0, 255, 255, 0, 0, 0, 255};

int engineRPM = 0;
int vehicleSpeed = 0;
int transmissionSpeed = 0;
int i_count = 0;

// --------------------------------------------------
// Gear Logic
// --------------------------------------------------
float Ratiocalc = 0;
bool shiftActive = false;
int Gear = 0;
float Ratioact = 0;
bool shiftWasDownshift = false;
bool shiftWasUpshift = false;

bool lastUpState = false;
bool lastDownState = false;

float gearGain[6] = {0,1,1,1,1,1};

float gearRatio[6] = {
  0.0,
  3.483,   // 1st
  2.015,   // 2nd
  1.391,   // 3rd
  1.000,   // 4th
  0.806    // 5th
};

#define GEAR_HYSTERESIS 0.15

// --------------------------------------------------
// Throttle / Regen Variables
// --------------------------------------------------

bool throttleFault = false;
unsigned long throttleFaultClearTimer = 0;

#define THROTTLE_IDLE_THRESHOLD 100
#define THROTTLE_FAULT_CLEAR_TIME 1000   // ms

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

uint16_t regen_raw = 0;
uint16_t regen_scaled = 0;

bool invertRegen = false;


// --------------------------------------------------


// --------------------------------------------------
// CRC8 (OpenInverter spec)
// --------------------------------------------------
uint8_t computeCRC8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0x00;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }

  return crc;
}

// --------------------------------------------------
// Gear Calculation
// --------------------------------------------------
void calcGear()
{
  if ((engineRPM < 50) || (transmissionSpeed < 5))
    return;

  // Calculate ratio
  Ratiocalc =
    ((float)engineRPM * 2080) /
    (transmissionSpeed * 1667 * 4.3);

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
// --------------------------------------------------
// Update MIL / Temp / ODO frame
// --------------------------------------------------

uint8_t engTemp = 140;
bool oilPressure = true;

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
  CAN.write(msg);
  canTxCount++;
lastTxID = msg.id;
}


  void sendPCMFrames()
{
    CanMsg msg203(0x203,7,send203);
    CAN.write(msg203);
    canTxCount++;
    lastTxID = msg203.id;

    CanMsg msg215(0x215,8,send215);
    CAN.write(msg215);
    canTxCount++;
    lastTxID = msg215.id;

    CanMsg msg231(0x231,8,send231);
    CAN.write(msg231);
    canTxCount++;
    lastTxID = msg231.id;

    CanMsg msg240(0x240,8,send240);
    CAN.write(msg240);
    canTxCount++;
    lastTxID = msg240.id;

    CanMsg msg620(0x620,7,send620);
    CAN.write(msg620);
    canTxCount++;
    lastTxID = msg620.id;

    CanMsg msg630(0x630,8,send630);
    CAN.write(msg630);
    canTxCount++;
    lastTxID = msg630.id;

    CanMsg msg650(0x650,1,send650);
    CAN.write(msg650);
    canTxCount++;
    lastTxID = msg650.id;
}


//--------------------------------------------
// Serial debuging
//--------------------------------------------
void debugOutput()
{
  Serial.println();
  Serial.println("----- VCU STATUS -----");

  // Vehicle state
  Serial.print("RPM:");
  Serial.print(engineRPM);
  Serial.print("   Speed:");
  Serial.print(vehicleSpeed);
  Serial.print(" kmh   Gear:");
  Serial.print(Gear);
  Serial.print("   Shift:");
  Serial.println(shiftActive);

  Serial.println();

  // Gear ratios
  Serial.println("Ratios");
  Serial.print("Calc:");
  Serial.print(Ratiocalc,3);
    Serial.print("Gear:");
Serial.print(Gear);
Serial.print("   RatioCalc:");
Serial.print(Ratiocalc,3);
Serial.print("   RatioAct:");
Serial.println(Ratioact,3);

  Serial.println();

  // Throttle
  Serial.println("Throttle");
  Serial.print("Raw1:");
  Serial.print(throttle1_raw);
  Serial.print("  Raw2:");
  Serial.print(throttle2_raw);

  Serial.print("   Scaled1:");
  Serial.print(throttle1_scaled);
  Serial.print("  Scaled2:");
  Serial.print(throttle2_scaled);

  Serial.print("   Diff:");
  Serial.println(throttle_diff);

  Serial.println();
Serial.println("Regen");

Serial.print("Raw:");
Serial.print(regen_raw);

Serial.print("   Scaled:");
Serial.print(regen_scaled);

Serial.print("   CAN:");
Serial.println(regenPreset);

  Serial.println();

  // Inputs
  Serial.println("Inputs");
  Serial.print("UpBtn:");
  Serial.print(upBtn.stableState);
  Serial.print("   DownBtn:");
  Serial.print(downBtn.stableState);
  Serial.print("   Neutral:");
  Serial.println(gearNBtn.stableState);

  Serial.println();

  // Outputs
  Serial.println("Outputs");
  Serial.print("CruiseTarget:");
  Serial.print(cruiseTarget);
  Serial.print("   ODOus:");
  Serial.println(ODOus);

  Serial.println("----------------------");
}

// --------------------------------------------------
// OpenInverter Command (10ms – LITTLE ENDIAN)
// --------------------------------------------------
void sendOpenInverterCommand()
{
  // ----- Read throttle -----

throttle1_raw = analogRead(throttle1Pin);
throttle2_raw = analogRead(throttle2Pin);


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
        0,
        4095);

throttle2_scaled =
    map(throttle2_raw,
        throttle2Min,
        throttle2Max,
        0,
        4095);

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
    if (throttleAtIdle)
    {
        if (millis() - throttleFaultClearTimer > THROTTLE_FAULT_CLEAR_TIME)
        {
            throttleFault = false;
        }
    }
    else
    {
        // reset timer if not idle
        throttleFaultClearTimer = millis();
    }

    // FORCE OUTPUT TO ZERO while fault is active
    throttle1_scaled = 0;
    throttle2_scaled = 0;
}
else
{
    // keep timer aligned when no fault present
    throttleFaultClearTimer = millis();
}

// Values used in CAN frame
uint16_t pot = throttle1_scaled;
uint16_t pot2 = throttle2_scaled;

// ----- Read regen -----
regen_raw = analogRead(regenPin);

// scale to 12-bit
regen_scaled = regen_raw * 4;

// optional inversion
if (invertRegen)
  regen_scaled = 4095 - regen_scaled;

// clamp
regen_scaled = constrain(regen_scaled, 0, 4095);

// optional deadzone
if (regen_scaled < 50)
  regen_scaled = 0;

// convert to %
regenPreset = regen_scaled >> 5;   // fast approx
if (regenPreset > 100)
  regenPreset = 100;


  // -----------------------------
// Update gear estimate when not shifting
// -----------------------------
if (!shiftActive)
{
  calcGear();
}


// -----------------------------
// Shift logic (FINAL VERSION)
// -----------------------------

uint16_t throttleAvg = (throttle1_scaled + throttle2_scaled) / 2;
bool throttleActive = throttleAvg > 200;

bool upPressed =
  upBtn.stableState && gearNBtn.stableState;

bool downPressed =
  downBtn.stableState && gearNBtn.stableState;
  
// edge detect (prevent retrigger during active shift)
bool upEdge = upPressed && !lastUpState && !shiftActive;
bool downEdge = downPressed && !lastDownState && !shiftActive;

// latch gear at start of shift
static int latchedGear = 0;

if (upEdge || downEdge)
{
  calcGear();
  latchedGear = Gear;
  shiftActive = true;
  shiftStartTime = millis();

  shiftWasDownshift = downEdge && !upEdge;
shiftWasUpshift = upEdge && !downEdge;
}
// -----------------------------
// Determine target gear ratio
// -----------------------------

float baseTarget = 0;
float targetRatio = 0;
bool cruiseActive = false;
uint8_t canio = 0;

if (shiftActive)
{
  if (shiftWasUpshift && latchedGear < 5)
  {
    targetRatio = gearRatio[latchedGear + 1];
    cruiseActive = true;
  }

  else if (shiftWasDownshift && latchedGear > 1)
  {
    targetRatio = gearRatio[latchedGear - 1];
    cruiseActive = true;
  }

  // calculate AFTER ratio is chosen
  if (targetRatio > 0)
  {
    baseTarget =
      ((targetRatio * transmissionSpeed * 1667 * 4.3) / 2080);
  }
}

// -----------------------------
// Shift END logic (time-based)
// -----------------------------
if (shiftActive)
{
  unsigned long shiftTime = millis() - shiftStartTime;

 bool isDownshift = shiftWasDownshift;

  unsigned long minTime =
    isDownshift ? DOWNSHIFT_MIN_TIME : UPSHIFT_MIN_TIME;

  bool minTimeElapsed = shiftTime > minTime;
  bool maxTimeExceeded = shiftTime > SHIFT_MAX_TIME;

  bool neutralExitAllowed = shiftTime > NEUTRAL_EXIT_DELAY;

  bool neutralLost = !gearNBtn.stableState;

 

  // End conditions:
  // - driver releases AFTER min time
  
  // - neutral lost AFTER delay (gear likely engaged)
  // - safety timeout

  if ((minTimeElapsed && !upPressed && !downPressed) ||
     
      (neutralExitAllowed && neutralLost) ||
      maxTimeExceeded)
  {
    shiftActive = false;
  }
}

// -----------------------------
// Update button states
// -----------------------------

// apply gain safely
int g = constrain(Gear, 1, 5);
cruiseTarget = baseTarget * gearGain[g];

  // store button state for next loop
  lastUpState = upPressed;
  lastDownState = downPressed;

  cruiseTarget = constrain(cruiseTarget, 0, 16383);

  if (cruiseActive)
    canio |= (1 << 0);   // cruise bit24

    // -----------------------------
// Unload drivetrain during shift
// -----------------------------
if (shiftActive)
{
    pot = 0;
    pot2 = 0;
    regenPreset = 0;
}

  // ----- Rolling counters -----
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

  for (int i = 0; i < 7; i++)
    payload[i] = (msg64 >> (8 * i)) & 0xFF;

  payload[7] = computeCRC8(payload, 7);

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

  if (!CAN.begin(CanBitRate::BR_500k))
  {
    Serial.println("CAN init failed");
    while (1);
  }

  updateMIL();
  engineRPM = 2000;   // temporary RPM to wake EPS
  vehicleSpeed = 0;

  Serial.println("RX8 VCU – Nano R4 Ready");
}

// --------------------------------------------------

void loop()
{


  updateDebounce(upBtn);
  updateDebounce(downBtn);
  updateDebounce(gearNBtn);

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

  //while (now - lastDashUpdate >= 50)
 // {
  //  lastDashUpdate += 50;
    
 // }
  
  while (now - lastPCMUpdate >= 75)
  {
    lastPCMUpdate += 75;
    sendPCMFrames();
    updatePCM();
  }
  
while (now - lastDebugUpdate >= 100)
{
    lastDebugUpdate += 100;

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
        engineRPM = (int)((float)rawRpm * 3.85);
    else
        engineRPM = 9000;
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
if (millis() - lastInverterMessage > 500)
{
    inverterSeen = false;
    engineRPM = 0;
}

  if (Serial.available())
  {
    serialConsoleTask();
  }

  //-------------------------------------------------------
  //Output fet control
  //-------------------------------------------------------
  // Oil Pump Control
if (inverterSeen && engineRPM > 100)
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
