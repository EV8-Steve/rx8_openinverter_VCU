# RX-8 Electric VCU (Arduino Nano R4)

## Overview

This project implements a **Vehicle Control Unit (VCU)** based on an
**Arduino Nano R4** to interface between:

-   Mazda RX‑8 instrument cluster
-   OpenInverter running **stm32‑foc v5.40.R**
-   Vehicle sensors and control inputs

The VCU reads analog and digital inputs, processes vehicle state, and
sends CAN messages to both the inverter and the RX‑8 dashboard.

### Primary Functions

-   Dual-channel throttle input processing
-   Gear‑based rev‑matching shift assist
-   CAN control of OpenInverter
-   CAN emulation for RX‑8 dashboard
-   Auxiliary outputs (oil pump)

------------------------------------------------------------------------

# System Architecture

    Vehicle Sensors
          │
          ▼
    Arduino Nano R4 (VCU)
          │
          ├── CAN → OpenInverter (motor control)
          │
          └── CAN → RX‑8 Dashboard

------------------------------------------------------------------------

# Hardware

## Main Controller

**Arduino Nano R4**

Features used:

-   Native CAN controller
-   10‑bit ADC
-   Digital GPIO
-   millis() non‑blocking timers
-   5V logic system

------------------------------------------------------------------------

## Power System

Input supply:

    10–20V Automotive Input
          │
          ▼
    TVS Diode Protection
          │
          ▼
    Fuse
          │
          ▼
    Buck Converter (LM2596)
          │
          ▼
    Stable 5V Rail

The 5V rail powers:

-   Nano R4
-   CAN transceiver
-   throttle sensors
-   digital inputs

Filtering includes bulk capacitors and 100nF decoupling.

------------------------------------------------------------------------

# CAN Network

## CAN Transceiver

**TJA1050**

  MCU   TJA1050
  ----- ---------
  D10   TXD
  D13   RXD

Bus:

-   CANH
-   CANL
-   Optional 120Ω termination

Bus speed:

    500 kbps

------------------------------------------------------------------------

# Inputs

## Throttle Pedal (Dual Channel)

  Input            Pin
  ---------------- -----
  Throttle Pot 1   A0
  Throttle Pot 2   A1

Voltage characteristics:

  Pot    Behaviour
  ------ -----------------
  Pot1   0‑5V increasing
  Pot2   5‑0V inverted

ADC scaling:

    10‑bit ADC → 12‑bit (0‑4095)

Second channel inversion:

    pot2 = 4095 - (ADC × 4)

Safety checks handled by OpenInverter firmware.

------------------------------------------------------------------------

## Digital Inputs

Each input uses **100kΩ pulldown resistors**.

  Pin   Function
  ----- --------------
  D4    Gear UP
  D5    Gear DOWN
  D7    Gear Neutral

Inputs are debounced in software (15ms).

------------------------------------------------------------------------

# Outputs

## Oil Pump Control

Output:

    D6

Driver:

    Logic level N‑MOSFET
    Low‑side switching

Includes:

-   100Ω gate resistor
-   100k gate pulldown
-   flyback diode

Logic:

    If engineRPM ≤ 100
        start timer
    If timer ≥ 3000 cycles
        disable pump
    Else
        pump ON

------------------------------------------------------------------------

# CAN Messaging

## OpenInverter Control Frame

  Parameter     Value
  ------------- ---------------
  CAN ID        0x300
  Update Rate   10ms
  Endianness    Little‑endian

Payload layout:

  Bits    Field
  ------- ---------------
  0‑11    throttle pot1
  12‑23   throttle pot2
  24‑29   CANIO
  30‑31   counter1
  32‑45   cruise target
  46‑47   counter2
  48‑55   regen preset
  56‑63   CRC

CRC parameters:

    CRC‑8
    Polynomial: 0x07
    Initial: 0x00
    No reflection
    No final XOR

Rolling counters: 2‑bit counters increment every frame.

------------------------------------------------------------------------

## RX‑8 Dashboard Message

  Parameter     Value
  ------------- ------------
  CAN ID        0x201
  Update Rate   20ms
  Endianness    Big‑endian

Fields:

  Bytes   Data
  ------- ---------------
  0‑1     Engine RPM
  4‑5     Vehicle speed

------------------------------------------------------------------------

# Gear Detection Logic

Gear ratio estimated from:

    engineRPM
    vehicleSpeed

Formula:

    ratio = (engineRPM × tyre_circumference) /
            (vehicleSpeed × 1667 × final_drive)

Example ratios:

  Gear   Ratio
  ------ -------
  1      3.483
  2      2.015
  3      1.391
  4      1.000
  5      0.806

These determine:

    Ratioup
    Ratiodown

------------------------------------------------------------------------

# Shift Assist System

Implements **rev‑matching gear changes**.

Operation:

1.  Driver presses UP or DOWN button
2.  Current gear ratio calculated
3.  Ratio is **latched**
4.  While button held:

```{=html}
<!-- -->
```
    targetRPM = latched_ratio × vehicleSpeed

Vehicle speed continues updating so the system keeps revs matched to
transmission speed.

Button release resets the shift system.

------------------------------------------------------------------------

# Control Loop Timing

  Task                   Period
  ---------------------- ------------
  OpenInverter control   10 ms
  Dashboard update       20 ms
  Input debounce         continuous

Scheduling uses `millis()` (non‑blocking).

------------------------------------------------------------------------

# Software Structure

Core functions:

    updateDebounce()
    calcGear()
    updatePCM()
    sendOpenInverterCommand()
    computeCRC8()

Main loop flow:

    Read inputs
     → Update dashboard
     → Send inverter control frame
     → Process incoming CAN
     → Update outputs

------------------------------------------------------------------------

# Responsibility Split

## VCU (Arduino)

Handles:

-   sensor reading
-   shift assist logic
-   CAN frame formatting
-   dashboard emulation
-   auxiliary outputs

## OpenInverter

Handles:

-   throttle safety
-   torque control
-   current limits
-   ramp limits
-   motor control

------------------------------------------------------------------------

# Hardware Summary

Major components:

  Component         Purpose
  ----------------- ----------------------
  Arduino Nano R4   Vehicle Control Unit
  TJA1050           CAN transceiver
  LM2596            DC‑DC converter
  Logic MOSFET      output switching
  TVS diode         input protection

------------------------------------------------------------------------

# Key Design Features

-   deterministic CAN timing
-   rev‑matching shift assist
-   dual‑channel throttle support
-   automotive power protection
-   modular architecture

------------------------------------------------------------------------

# Project Status

Current state:

-   CAN messaging functional
-   shift assist logic implemented
-   hardware architecture defined
-   KiCad schematic development in progress
-   firmware ready for testing
