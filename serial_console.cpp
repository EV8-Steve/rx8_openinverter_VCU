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

  // ---------------- HELP ----------------
  if (cmd == "help")
  {
    Serial.println();
    Serial.println("---- COMMANDS ----");
    Serial.println("settings        - show all settings");
    Serial.println("save            - save settings");
    Serial.println("defaults        - load defaults");
    Serial.println("debug on/off    - toggle debug");
    Serial.println("debug           - show debug state");
    Serial.println("set t1inv 0/1   - throttle1 invert");
    Serial.println("set t2inv 0/1   - throttle2 invert");
    Serial.println("set tdiff N     - throttle diff limit");
    Serial.println("set gain G V    - gear gain (G=1-5)");
    Serial.println("------------------");
    Serial.println();
  }

  // ---------------- SETTINGS ----------------
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

  // ---------------- DEBUG ----------------
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

  else if (cmd == "debug")
  {
    Serial.print("Debug: ");
    Serial.println(debugEnabled ? "ON" : "OFF");
  }

  // ---------------- SET COMMANDS ----------------
  else if (cmd.startsWith("set "))
  {
    processSetCommand(cmd);
  }

  else
  {
    Serial.println("Unknown command (type 'help')");
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
