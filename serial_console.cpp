#include <Arduino.h>
#include "serial_console.h"
#include "vcu_settings.h"

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

// --------------------------------------------------
void serialConsoleInit()
{
  inputBuffer.reserve(64);
}

// --------------------------------------------------
void serialConsoleTask()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (inputBuffer.length() > 0)
      {
        handleCommand(inputBuffer);
        inputBuffer = "";
      }
    }
    else
    {
      inputBuffer += c;
    }
  }
}

// --------------------------------------------------
// MAIN COMMAND HANDLER
// --------------------------------------------------
void handleCommand(String cmd)
{
    cmd.trim();

    if (cmd == "help")
    {
        Serial.println();
        Serial.println("---- COMMANDS ----");
        Serial.println("help");
        Serial.println("settings");
        Serial.println("save");
        Serial.println("defaults");
        Serial.println("can");
        Serial.println("debug");
        Serial.println("debug on");
        Serial.println("debug off");
        Serial.println("candump on");
        Serial.println("candump off");
        Serial.println("learnthrottle");
        Serial.println("next");
        Serial.println("set t1inv 0/1");
        Serial.println("set t2inv 0/1");
        Serial.println("set tdiff N");
        Serial.println("set t1min N");
        Serial.println("set t1max N");
        Serial.println("set t2min N");
        Serial.println("set t2max N");
        Serial.println("set gain G V");
        Serial.println("------------------");
    }

    else if (cmd == "settings")
    {
        settingsPrint();
    }


    else if (cmd == "save")
    {
        settingsSave();
    }

    else if (cmd == "defaults")
    {
        settingsDefaults();
    }

    else if (cmd == "debug")
    {
        Serial.print("Debug: ");
        Serial.println(debugEnabled ? "ON" : "OFF");
    }

    else if (cmd == "can")
{
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
}
else if (cmd == "candump on")
{
    canDumpEnabled = true;
    Serial.println("CAN dump ENABLED");
}

else if (cmd == "candump off")
{
    canDumpEnabled = false;
    Serial.println("CAN dump DISABLED");
}

    else if (cmd == "debug on")
    {
        debugEnabled = true;
        Serial.println("Debug ENABLED");
    }

    else if (cmd == "debug off")
    {
        debugEnabled = false;
        Serial.println("Debug DISABLED");
    }

    else if (cmd == "learnthrottle")
    {
        throttleCalMode = true;
        throttleCalWaitingMax = false;

        Serial.println();
        Serial.println("Throttle calibration");
        Serial.println("Release pedal.");
        Serial.println("Type NEXT");
    }

    else if (cmd == "next" && throttleCalMode)
    {
        if (!throttleCalWaitingMax)
        {
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
        }
        else
        {
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

    else if (cmd.startsWith("set "))
    {
        processSetCommand(cmd);
    }

    else
    {
        Serial.println("Unknown command");
    }
}
// --------------------------------------------------
// SET COMMAND PARSER
// --------------------------------------------------
void processSetCommand(String cmd)
{
  // ---------- THROTTLE INVERSION ----------
  if (cmd.startsWith("set t1inv "))
  {
    int val = cmd.substring(10).toInt();
    invertThrottle1 = (val != 0);

    Serial.print("Throttle1 invert: ");
    Serial.println(invertThrottle1 ? "ON" : "OFF");
  }

  else if (cmd.startsWith("set t2inv "))
  {
    int val = cmd.substring(10).toInt();
    invertThrottle2 = (val != 0);

    Serial.print("Throttle2 invert: ");
    Serial.println(invertThrottle2 ? "ON" : "OFF");
  }

  // ---------- THROTTLE DIFF ----------
  else if (cmd.startsWith("set tdiff "))
  {
    int val = cmd.substring(10).toInt();

    if (val > 0 && val < 2000)
    {
      throttleMaxDiff = val;

      Serial.print("Throttle diff max set to: ");
      Serial.println(throttleMaxDiff);
    }
    else
    {
      Serial.println("Invalid value (1–2000)");
    }
  }
  else if (cmd.startsWith("set t1min "))
{
    throttle1Min = cmd.substring(10).toInt();

    Serial.print("Throttle1 Min = ");
    Serial.println(throttle1Min);
}
else if (cmd.startsWith("set t1max "))
{
    throttle1Max = cmd.substring(10).toInt();

    Serial.print("Throttle1 Max = ");
    Serial.println(throttle1Max);
}
else if (cmd.startsWith("set t2min "))
{
    throttle2Min = cmd.substring(10).toInt();

    Serial.print("Throttle2 Min = ");
    Serial.println(throttle2Min);
}
else if (cmd.startsWith("set t2max "))
{
    throttle2Max = cmd.substring(10).toInt();

    Serial.print("Throttle2 Max = ");
    Serial.println(throttle2Max);
}

  // ---------- GEAR GAIN ----------
  else if (cmd.startsWith("set gain "))
  {
    int gear = cmd.substring(9, 10).toInt();
    float val = cmd.substring(11).toFloat();

    if (gear >= 1 && gear <= 5)
    {
      if (val > 0.5 && val < 1.5)
      {
        gearGain[gear] = val;

        Serial.print("Gear ");
        Serial.print(gear);
        Serial.print(" gain = ");
        Serial.println(val, 3);
      }
      else
      {
        Serial.println("Gain out of range (0.5–1.5)");
      }
    }
    else
    {
      Serial.println("Invalid gear (1–5)");
    }
  }

  else
  {
    Serial.println("Unknown SET command");
  }
}
