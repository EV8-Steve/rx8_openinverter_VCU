#include <Arduino.h>
#include "serial_console.h"
#include "vcu_settings.h"
#include <stdio.h>
#include <math.h>

// --------------------------------------------------
// External variables from main
// --------------------------------------------------
extern bool invertThrottle1;
extern bool invertThrottle2;
extern uint16_t throttleMaxDiff;
extern float gearGain[6];
extern bool debugEnabled;
extern bool throttleCalMode;
extern bool throttleCalWaitingMax;
extern int engineRPM;
extern int vehicleSpeed;
extern int engTemp;
extern int Gear;
extern float Ratiocalc;
extern int motorRPM;
extern float Ratioact;
extern float wheelRPM;
extern float gearboxOutputRPM;
extern uint16_t throttle1_scaled;
extern uint16_t throttle2_scaled;
extern uint16_t throttle_diff;
extern uint8_t regenPreset;
extern bool inverterSeen;
extern bool throttleFault;
extern bool clutchPressed;
extern bool ignitionOn;
extern bool reverseSelected;
extern float ratioAverage[6];
extern uint32_t ratioCount[6];
extern float ratioSum[6];
extern bool steadyState;
extern unsigned long steadyStateTimer;
extern bool ratioLearningActive;
extern int learnGear;
extern bool ratioLearningEnabled;
extern bool loggingEnabled;
extern bool shiftTorqueCut;
extern bool shiftCruiseEnable;
extern uint8_t SHIFT_TORQUE_RAMP_DOWN;
extern uint8_t SHIFT_TORQUE_RAMP_UP;


enum ShiftDirection {
  SHIFT_UP,
  SHIFT_DOWN
};


extern ShiftDirection shiftDirection;
extern uint32_t pcmOverruns;

extern bool upButton;
extern bool downButton;
extern bool neutralButton;

extern bool brakePressed;

extern float frontWheelSpeedKmh;
extern float rearWheelSpeedKmh;
extern float wheelSlipKmh;
extern float wheelSlipPercent;
extern float TC_KP[6];
extern float TC_KI[6];
extern float TC_KD[6];

extern bool tractionControlEnabled;
extern bool tractionControlActive;
extern uint8_t tcTorquePercent;

extern uint16_t cruiseTarget;
extern uint16_t shiftCruiseTarget;
extern int latchedGear;
extern int shiftTargetGear;

extern float shiftTargetRatio;
extern uint16_t shiftBaseTarget;

enum ShiftState {
  SHIFT_IDLE,
  SHIFT_TORQUE_CUT,
  SHIFT_REV_MATCH,
};

extern ShiftState shiftState;

// CAN diagnostics
extern uint32_t canTxCount;
extern uint32_t canRxCount;

extern uint32_t txPerSecond;
extern uint32_t rxPerSecond;

extern uint32_t lastTxID;
extern uint32_t lastRxID;

extern uint16_t learnedT1Min;
extern uint16_t learnedT1Max;

extern uint16_t learnedT2Min;
extern uint16_t learnedT2Max;

extern uint16_t throttle1_raw;
extern uint16_t throttle2_raw;

extern uint16_t throttle1Min;
extern uint16_t throttle1Max;

extern uint16_t throttle2Min;
extern uint16_t throttle2Max;

extern bool canDumpEnabled;

// --------------------------------------------------
static String inputBuffer;

// Forward declarations
void handleCommand(String cmd);
void processSetCommand(String cmd);

///////helpers/////////


void printYesNo(bool state) {
  Serial.println(state ? "YES" : "NO");
}

//--------------------------------------------------
// Dashboard helpers
//--------------------------------------------------

void clearScreen() {
  // ANSI clear screen + cursor home
  Serial.print("\033[2J");
  Serial.print("\033[H");
}

void printLine(const char* label) {
  Serial.print(label);

  int spaces = 24 - strlen(label);

  while (spaces-- > 0)
    Serial.print(' ');
}

void printBool(bool state) {
  Serial.println(state ? "YES" : "NO");
}


// --------------------------------------------------
void serialConsoleInit() {
  inputBuffer.reserve(64);
}

// --------------------------------------------------
void serialConsoleTask() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

// --------------------------------------------------
// MAIN COMMAND HANDLER
// --------------------------------------------------
void handleCommand(String cmd) {
  cmd.trim();

  if (cmd == "help") {
    Serial.println();
    Serial.println("--------------- COMMANDS ---------------");
    Serial.println("help                 settings");
    Serial.println("save                 defaults");
    Serial.println("can                  debug");
    Serial.println("debug on             debug off");
    Serial.println("candump on           candump off");
    Serial.println("learnthrottle        next");
    Serial.println("set t1inv 0/1        set t2inv 0/1");
    Serial.println("set tdiff N          set t1min N");
    Serial.println("set t1max N          set t2min N");
    Serial.println("set t2max N          gains");
    Serial.println("set g1 V             set g2 V");
    Serial.println("set g3 V             set g4 V");
    Serial.println("set g5 V");
    Serial.println("set rampdown <1-100 %/10ms>");
    Serial.println("set rampup   <1-100 %/10ms>");
    Serial.println("ratio                ratio clear");
    Serial.println("ratiolearn           ratiolearn on");
    Serial.println("ratiolearn off       log");
    Serial.println("log on               log off");
    Serial.println("tcgains              TC PID gains");
    Serial.println("set tckp1 V           set tckp2 V");
    Serial.println("set tckp3 V           set tckp4 V");
    Serial.println("set tckp5 V");
    Serial.println("set tcki1 V           set tcki2 V");
    Serial.println("set tcki3 V           set tcki4 V");
    Serial.println("set tcki5 V");
    Serial.println("set tckd1 V           set tckd2 V");
    Serial.println("set tckd3 V           set tckd4 V");
    Serial.println("set tckd5 V");
    Serial.println("----------------------------------------");
  }

  else if (cmd == "settings") {
    settingsPrint();
  }


  else if (cmd == "save") {
    settingsSave();
  }

  else if (cmd == "defaults") {
    settingsDefaults();
  }

  else if (cmd == "debug") {
    Serial.print("Debug: ");
    Serial.println(debugEnabled ? "ON" : "OFF");
  }

  else if (cmd == "can") {
    Serial.println();

    Serial.println("------ CAN STATUS ------");

    Serial.print("TX Count : ");
    Serial.println(canTxCount);

    Serial.print("RX Count : ");
    Serial.println(canRxCount);

    Serial.println();

    Serial.print("TX/sec   : ");
    Serial.println(txPerSecond);

    Serial.print("RX/sec   : ");
    Serial.println(rxPerSecond);

    Serial.println();

    Serial.print("Last TX  : 0x");
    Serial.println(lastTxID, HEX);

    Serial.print("Last RX  : 0x");
    Serial.println(lastRxID, HEX);

    Serial.println();

    if (txPerSecond == 0)
      Serial.println("WARNING: No CAN TX");

    if (rxPerSecond == 0)
      Serial.println("WARNING: No CAN RX");

    if (txPerSecond > 0 && rxPerSecond > 0)
      Serial.println("CAN BUS ACTIVE");

    Serial.println("------------------------");
  } else if (cmd == "candump on") {
    canDumpEnabled = true;
    Serial.println("CAN dump ENABLED");
  }

  else if (cmd == "candump off") {
    canDumpEnabled = false;
    Serial.println("CAN dump DISABLED");
  }

  else if (cmd == "debug on") {
    debugEnabled = true;
    Serial.println("Debug ENABLED");
  }

  else if (cmd == "debug off") {
    debugEnabled = false;
    Serial.println("Debug DISABLED");
  }

  else if (cmd == "log") {
    Serial.print("Logging: ");
    Serial.println(loggingEnabled ? "ON" : "OFF");
  } else if (cmd == "log on") {
    loggingEnabled = true;

    Serial.println("Logging ENABLED");


    Serial.println(
      "Time,"
      "MotorRPM,"
      "EngineRPM,"
      "VehicleSpeed,"
      "WheelRPM,"
      "GearboxRPM,"
      "Gear,"
      "LatchedGear,"
      "LearnGear,"
      "TargetGear,"
      "ShiftState,"
      "ShiftDir,"
      "RatioCalc,"
      "RatioTarget,"
      "RatioErr,"
      "ShiftBase,"
      "ShiftTarget,"
      "MotorErr,"
      "T1Raw,"
      "T2Raw,"
      "T1,"
      "T2,"
      "ThrottleDiff,"
      "ThrottlePct,"
      "Regen,"
      "Brake,"
      "Clutch,"
      "Neutral,"
      "Up,"
      "Down,"
      "Ignition,"
      "Reverse,"
      "InvSeen,"
      "ThrottleFault,"
      "TorqueCut,"
      "CruiseEnable,");

    Serial.println(
      "CAN_TX,"
      "CAN_RX,"
      "TXperSec,"
      "RXperSec,"
      "PCMOverruns,"
      "LearnEnabled,"
      "LearnActive,"
      "FrontWheelKmh,"
      "RearWheelKmh,"
      "SlipKmh,"
      "SlipPercent,"
      "TCEnabled,"
      "TCActive,"
      "TcTorquePercent");
  }

  else if (cmd == "log off") {
    loggingEnabled = false;

    Serial.println("Logging DISABLED");
  }


  else if (cmd == "learnthrottle") {
    throttleCalMode = true;
    throttleCalWaitingMax = false;

    Serial.println();
    Serial.println("Throttle calibration");
    Serial.println("Release pedal.");
    Serial.println("Type NEXT");
  }

  else if (cmd == "next" && throttleCalMode) {
    if (!throttleCalWaitingMax) {
      learnedT1Min = throttle1_raw;
      learnedT2Min = throttle2_raw;

      throttleCalWaitingMax = true;

      Serial.println();
      Serial.println("Idle captured");

      Serial.print("T1=");
      Serial.println(learnedT1Min);

      Serial.print("T2=");
      Serial.println(learnedT2Min);

      Serial.println();
      Serial.println("Press pedal fully.");
      Serial.println("Type NEXT");
    } else {
      learnedT1Max = throttle1_raw;
      learnedT2Max = throttle2_raw;

      throttle1Min = learnedT1Min;
      throttle1Max = learnedT1Max;

      throttle2Min = learnedT2Min;
      throttle2Max = learnedT2Max;

      throttleCalMode = false;

      Serial.println();
      Serial.println("Full throttle captured");

      Serial.print("T1=");
      Serial.println(learnedT1Max);

      Serial.print("T2=");
      Serial.println(learnedT2Max);

      Serial.println();
      Serial.println("Calibration updated.");
      Serial.println("Type SAVE to store permanently.");
    }
  }

  else if (cmd == "ratio") {
    Serial.println();
    Serial.println("Learned Gear Ratios");
    Serial.println("------------------------------");

    for (int i = 1; i <= 5; i++) {
      Serial.print("Gear ");
      Serial.print(i);
      Serial.print(": ");

      Serial.print(ratioAverage[i], 4);

      Serial.print("   Samples: ");

      Serial.println(ratioCount[i]);
    }

    Serial.println();
  }

  else if (cmd == "ratio clear") {
    for (int i = 0; i < 6; i++) {
      ratioSum[i] = 0.0f;
      ratioCount[i] = 0;
      ratioAverage[i] = 0.0f;
    }

    steadyState = false;
    steadyStateTimer = 0;

    Serial.println("Ratio learning cleared.");
  }

  else if (cmd == "gains") {
    Serial.println();

    for (int i = 1; i <= 5; i++) {
      Serial.print("Gear ");
      Serial.print(i);
      Serial.print(": ");

      Serial.println(gearGain[i], 3);
    }

    Serial.println();
  }



  else if (cmd == "ratiolearn") {
    Serial.print("Ratio learning: ");
    Serial.println(ratioLearningEnabled ? "ON" : "OFF");
  }

  else if (cmd == "ratiolearn on") {
    ratioLearningEnabled = true;
    Serial.println("Ratio learning ENABLED");
  }

  else if (cmd == "ratiolearn off") {
    ratioLearningEnabled = false;
    Serial.println("Ratio learning DISABLED");
  }

  else if (cmd == "tcgains") {

    Serial.println();
    Serial.println("Traction Control PID Gains");
    Serial.println("--------------------------");

    for (int i = 1; i <= 5; i++) {

      Serial.print("Gear ");
      Serial.print(i);

      Serial.print("  Kp=");
      Serial.print(TC_KP[i], 3);

      Serial.print("  Ki=");
      Serial.print(TC_KI[i], 3);

      Serial.print("  Kd=");
      Serial.println(TC_KD[i], 3);
    }

    Serial.println();
  }

  else if (cmd.startsWith("set ")) {
    processSetCommand(cmd);
  }



  else {
    Serial.println("Unknown command");
  }
}
// --------------------------------------------------
// SET COMMAND PARSER
// --------------------------------------------------
void processSetCommand(String cmd) {
  // ---------- THROTTLE INVERSION ----------
  if (cmd.startsWith("set t1inv ")) {
    int val = cmd.substring(10).toInt();
    invertThrottle1 = (val != 0);
    settingsSave();

    Serial.print("Throttle1 invert: ");
    Serial.println(invertThrottle1 ? "ON" : "OFF");
  }

  else if (cmd.startsWith("set t2inv ")) {
    int val = cmd.substring(10).toInt();
    invertThrottle2 = (val != 0);
    settingsSave();

    Serial.print("Throttle2 invert: ");
    Serial.println(invertThrottle2 ? "ON" : "OFF");
  }

  // ---------- THROTTLE DIFF ----------
  else if (cmd.startsWith("set tdiff ")) {
    int val = cmd.substring(10).toInt();

    if (val > 0 && val < 2000) {
      throttleMaxDiff = val;
      settingsSave();

      Serial.print("Throttle diff max set to: ");
      Serial.println(throttleMaxDiff);
    } else {
      Serial.println("Invalid value (1–2000)");
    }
  } else if (cmd.startsWith("set t1min ")) {
    throttle1Min = cmd.substring(10).toInt();
    settingsSave();

    Serial.print("Throttle1 Min = ");
    Serial.println(throttle1Min);
  } else if (cmd.startsWith("set t1max ")) {
    throttle1Max = cmd.substring(10).toInt();
    settingsSave();

    Serial.print("Throttle1 Max = ");
    Serial.println(throttle1Max);
  } else if (cmd.startsWith("set t2min ")) {
    throttle2Min = cmd.substring(10).toInt();
    settingsSave();

    Serial.print("Throttle2 Min = ");
    Serial.println(throttle2Min);
  } else if (cmd.startsWith("set t2max ")) {
    throttle2Max = cmd.substring(10).toInt();
    settingsSave();

    Serial.print("Throttle2 Max = ");
    Serial.println(throttle2Max);
  }

  // ---------- Shift GAIN ----------
  else if (cmd.startsWith("set g")) {
    int gear = cmd.substring(5, 6).toInt();
    float val = cmd.substring(7).toFloat();

    if (gear >= 1 && gear <= 5) {
      if (val >= 0.5f && val <= 1.5f) {
        gearGain[gear] = val;
        settingsSave();

        Serial.print("Gear ");
        Serial.print(gear);
        Serial.print(" gain = ");
        Serial.println(gearGain[gear], 3);
      } else {
        Serial.println("Gain out of range (0.5-1.5)");
      }
    } else {
      Serial.println("Invalid gear");
    }
  }
  // ---------- Torque Ramp Down ----------
  else if (cmd.startsWith("set rampdown")) {
    if (cmd == "set rampdown") {
      Serial.print("Torque Ramp Down : ");
      Serial.print(SHIFT_TORQUE_RAMP_DOWN);
      Serial.println(" %/10ms");
    } else {
      int val = cmd.substring(13).toInt();

      if (val >= 1 && val <= 100) {
        SHIFT_TORQUE_RAMP_DOWN = val;
        settingsSave();

        Serial.print("Torque Ramp Down set to ");
        Serial.print(SHIFT_TORQUE_RAMP_DOWN);
        Serial.println(" %/10ms");
      } else {
        Serial.println("Value out of range (1-100)");
      }
    }
  }
  // ---------- Torque Ramp Up ----------
  else if (cmd.startsWith("set rampup")) {
    if (cmd == "set rampup") {
      Serial.print("Torque Ramp Up : ");
      Serial.print(SHIFT_TORQUE_RAMP_UP);
      Serial.println(" %/10ms");
    } else {
      int val = cmd.substring(11).toInt();

      if (val >= 1 && val <= 100) {
        SHIFT_TORQUE_RAMP_UP = val;
        settingsSave();

        Serial.print("Torque Ramp Up set to ");
        Serial.print(SHIFT_TORQUE_RAMP_UP);
        Serial.println(" %/10ms");
      } else {
        Serial.println("Value out of range (1-100)");
      }
    }
  }
  // --------------------------------------------------
  // TC PID PROPORTIONAL GAIN
  // --------------------------------------------------

  else if (cmd.startsWith("set tckp")) {

    int gear = cmd.substring(8, 9).toInt();
    float val = cmd.substring(10).toFloat();

    if (gear >= 1 && gear <= 5) {

      if (val >= 0.0f && val <= 20.0f) {

        TC_KP[gear] = val;
        settingsSave();

        Serial.print("TC Gear ");
        Serial.print(gear);
        Serial.print(" Kp = ");
        Serial.println(TC_KP[gear], 3);

      } else {

        Serial.println("Kp out of range (0-20)");
      }

    } else {

      Serial.println("Invalid gear");
    }
  }


  // --------------------------------------------------
  // TC PID INTEGRAL GAIN
  // --------------------------------------------------

  else if (cmd.startsWith("set tcki")) {

    int gear = cmd.substring(8, 9).toInt();
    float val = cmd.substring(10).toFloat();

    if (gear >= 1 && gear <= 5) {

      if (val >= 0.0f && val <= 5.0f) {

        TC_KI[gear] = val;
        settingsSave();

        Serial.print("TC Gear ");
        Serial.print(gear);
        Serial.print(" Ki = ");
        Serial.println(TC_KI[gear], 3);

      } else {

        Serial.println("Ki out of range (0-5)");
      }

    } else {

      Serial.println("Invalid gear");
    }
  }


  // --------------------------------------------------
  // TC PID DERIVATIVE GAIN
  // --------------------------------------------------

  else if (cmd.startsWith("set tckd"))
   {
    int gear = cmd.substring(8, 9).toInt();
    float val = cmd.substring(10).toFloat();

    if (gear >= 1 && gear <= 5) {

      if (val >= 0.0f && val <= 5.0f) {

        TC_KD[gear] = val;
        settingsSave();

        Serial.print("TC Gear ");
        Serial.print(gear);
        Serial.print(" Kd = ");
        Serial.println(TC_KD[gear], 3);

      } else {

        Serial.println("Kd out of range (0-5)");
      }

    } else {

      Serial.println("Invalid gear");
    }
  } else {
    Serial.println("Unknown SET command");
  }
}

//--------------------------------------------
// Serial debuging
//--------------------------------------------
void debugOutput() {

  const char* shiftStateText = "UNKNOWN";

  switch (shiftState) {
    case SHIFT_IDLE:
      shiftStateText = "IDLE";
      break;

    case SHIFT_TORQUE_CUT:
      shiftStateText = "CUT";
      break;

    case SHIFT_REV_MATCH:
      shiftStateText = "MATCH";
      break;
  }
  static char screen[768];
  size_t len = 0;

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "\n\n"
                  "==============================================================\n"
                  "                    RX-8 EV VCU\n"
                  "==============================================================\n");

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "MRPM:%5d DRPM:%5d SPD:%3dkm/h TMP:%3dC\n",
                  motorRPM,
                  engineRPM,
                  vehicleSpeed,
                  engTemp);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "WR:%5.1f GO:%5.1f G:%d LG:%d TG:%d\n",
                  wheelRPM,
                  gearboxOutputRPM,
                  Gear,
                  latchedGear,
                  shiftTargetGear);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "WR:%5.1f GO:%5.1f G:%d LRN:%d LG:%d TG:%d\n",
                  wheelRPM,
                  gearboxOutputRPM,
                  Gear,
                  learnGear,
                  latchedGear,
                  shiftTargetGear);

  float ratioError = 0.0f;

  if (shiftState != SHIFT_IDLE) {
    ratioError = fabs(Ratiocalc - shiftTargetRatio);
  }

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "ERR:%1.3f\n",
                  ratioError);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "MERR:%4d\n",
                  motorRPM - shiftCruiseTarget);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "T1:%4d  T2:%4d  D:%3d  REG:%2d%%\n",
                  throttle1_scaled,
                  throttle2_scaled,
                  throttle_diff,
                  regenPreset);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "UP:%d DN:%d N:%d BR:%d CL:%d IGN:%d REV:%d\n",
                  upButton,
                  downButton,
                  neutralButton,
                  brakePressed,
                  clutchPressed,
                  ignitionOn,
                  reverseSelected);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "SH:%s BT:%4d RT:%4d CAN:%4d\n",
                  shiftStateText,
                  shiftBaseTarget,
                  shiftCruiseTarget,
                  cruiseTarget);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "TG:%d G1:%1.3f G2:%1.3f G3:%1.3f G4:%1.3f G5:%1.3f\n",
                  shiftTargetGear,
                  gearGain[1],
                  gearGain[2],
                  gearGain[3],
                  gearGain[4],
                  gearGain[5]);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "LEARN:%s ACTIVE:%s\n",
                  ratioLearningEnabled ? "ON" : "OFF",
                  ratioLearningActive ? "YES" : "NO");

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "R1:%lu R2:%lu R3:%lu R4:%lu R5:%lu\n",
                  ratioCount[1],
                  ratioCount[2],
                  ratioCount[3],
                  ratioCount[4],
                  ratioCount[5]);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "A:%1.3f %1.3f %1.3f %1.3f %1.3f\n",
                  ratioAverage[1],
                  ratioAverage[2],
                  ratioAverage[3],
                  ratioAverage[4],
                  ratioAverage[5]);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "INV:%d TF:%d TX:%3lu RX:%3lu PCM:%lu\n",
                  inverterSeen,
                  throttleFault,
                  txPerSecond,
                  rxPerSecond,
                  pcmOverruns);

  int debugGear = constrain(Gear, 1, 5);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "TC:%d ACT:%d TORQ:%3d SLIP:%5.1f Kp:%1.2f Ki:%1.2f Kd:%1.2f\n",
                  tractionControlEnabled,
                  tractionControlActive,
                  tcTorquePercent,
                  wheelSlipPercent,
                  TC_KP[debugGear],
                  TC_KI[debugGear],
                  TC_KD[debugGear]);

  len += snprintf(screen + len,
                  sizeof(screen) - len,
                  "==============================================================\n");

  Serial.print(screen);
}

void serialLogOutput() {
  Serial.print(millis());

  Serial.print(',');
  Serial.print(motorRPM);

  Serial.print(',');
  Serial.print(engineRPM);

  Serial.print(',');
  Serial.print(vehicleSpeed);

  Serial.print(',');
  Serial.print(wheelRPM, 2);

  Serial.print(',');
  Serial.print(gearboxOutputRPM, 2);

  Serial.print(',');
  Serial.print(Gear);

  Serial.print(',');
  Serial.print(latchedGear);

  Serial.print(',');
  Serial.print(learnGear);

  Serial.print(',');
  Serial.print(shiftTargetGear);

  Serial.print(',');
  Serial.print((int)shiftState);

  Serial.print(',');
  Serial.print((shiftDirection == SHIFT_UP) ? 'U' : 'D');

  Serial.print(',');
  Serial.print(Ratiocalc, 3);

  Serial.print(',');
  Serial.print(shiftTargetRatio, 3);

  Serial.print(',');
  Serial.print(fabs(Ratiocalc - shiftTargetRatio), 3);

  Serial.print(',');
  Serial.print(shiftBaseTarget);

  Serial.print(',');
  Serial.print(shiftCruiseTarget);

  Serial.print(',');
  Serial.print(motorRPM - shiftCruiseTarget);

  Serial.print(',');
  Serial.print(throttle1_raw);

  Serial.print(',');
  Serial.print(throttle2_raw);

  Serial.print(',');
  Serial.print(throttle1_scaled);

  Serial.print(',');
  Serial.print(throttle2_scaled);

  Serial.print(',');
  Serial.print(throttle_diff);

  Serial.print(',');
  Serial.print(throttle1_scaled / 20);  // or whatever your % calculation is

  Serial.print(',');
  Serial.print(regenPreset);

  Serial.print(',');
  Serial.print(brakePressed);

  Serial.print(',');
  Serial.print(clutchPressed);

  Serial.print(',');
  Serial.print(neutralButton);

  Serial.print(',');
  Serial.print(upButton);

  Serial.print(',');
  Serial.print(downButton);

  Serial.print(',');
  Serial.print(ignitionOn);

  Serial.print(',');
  Serial.print(reverseSelected);

  Serial.print(',');
  Serial.print(inverterSeen);

  Serial.print(',');
  Serial.print(throttleFault);

  Serial.print(',');
  Serial.print(shiftTorqueCut);

  Serial.print(',');
  Serial.print(shiftCruiseEnable);

  Serial.print(',');
  Serial.print(canTxCount);

  Serial.print(',');
  Serial.print(canRxCount);

  Serial.print(',');
  Serial.print(txPerSecond);

  Serial.print(',');
  Serial.print(rxPerSecond);

  Serial.print(',');
  Serial.print(pcmOverruns);

  Serial.print(',');
  Serial.print(ratioLearningEnabled);

  Serial.print(',');
  Serial.print(ratioLearningActive);

  Serial.print(',');
  Serial.print(frontWheelSpeedKmh);

  Serial.print(',');
  Serial.print(rearWheelSpeedKmh);

  Serial.print(',');
  Serial.print(wheelSlipKmh);

  Serial.print(',');
  Serial.print(wheelSlipPercent);

  Serial.print(',');
  Serial.print(tractionControlEnabled);

  Serial.print(',');
  Serial.print(tractionControlActive);

  Serial.print(',');
  Serial.println(tcTorquePercent);
}
