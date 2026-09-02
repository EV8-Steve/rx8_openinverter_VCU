# RX-8 Electric VCU

**Arduino Nano R4 based Vehicle Control Unit for the electric Mazda RX-8**

This project is an open-source Vehicle Control Unit (VCU) developed specifically for an electric-converted Mazda RX-8.

The VCU sits between the vehicle's original controls and instrumentation and an **OpenInverter STM32-Foc based motor controller**, providing the vehicle-level logic required to make the electric drivetrain behave like an integrated part of the car.

---

## Overview

The VCU is based around an **Arduino Nano R4** and provides the interface between:

* Mazda RX-8 driver controls and sensors
* Original RX-8 instrument cluster
* OpenInverter motor controller
* Manual gearbox and paddle-shift controls
* Auxiliary vehicle systems

The firmware handles vehicle inputs, drivetrain state, gear detection, shift assistance, traction control and CAN communications.

The current firmware is developed around:

**OpenInverter / STM32-Foc v5.40.R**

---

# Features

## Driver Controls

The VCU supports:

* Dual-channel accelerator pedal
* Paddle upshift
* Paddle downshift
* Neutral switch
* Brake input
* Reverse input
* Ignition input
* Clutch input

The accelerator pedal uses two independent channels and is checked for correct correlation before being passed to the inverter.

---

## Paddle Shift Assist

The RX-8 retains its manual gearbox.

The VCU provides an automatic clutchless shift-assistance system which controls motor torque and speed during gear changes.

A paddle press initiates the shift. The VCU then manages the remainder of the sequence automatically.

### Shift sequence

```text
             PADDLE INPUT
                  │
                  ▼
              SHIFT START
                  │
                  ▼
             TORQUE RAMP
                  │
                  │ Motor torque
                  │ reduced to zero
                  ▼
              NEUTRAL
                  │
                  ▼
             REV MATCH
                  │
                  │ Target RPM continuously
                  │ follows vehicle speed
                  ▼
            TARGET GEAR
                  │
                  ▼
          GEAR CONFIRMATION
                  │
                  ▼
              SHIFT DONE
```

The paddle does **not** need to remain pressed after the shift has been initiated.

### Rev matching

The target motor speed is calculated from:

* Vehicle speed
* Wheel RPM
* Gearbox output speed
* Target gear ratio
* Final drive ratio
* Configured shift gain

The target is continuously recalculated while the gearbox is in neutral.

This is important because the driver can pause during a shift and the VCU will continue to track the required motor speed as the vehicle slows or accelerates.

### Shift torque control

Torque reduction during a shift is performed using a configurable torque ramp.

Torque ramp values are expressed as:

**% change per 10 ms control cycle**

This makes the tuning parameters directly relate to the VCU's 10 ms inverter control loop.

Upshift and downshift behaviour can be tuned independently.

---

# Traction Control

The VCU now includes drivetrain traction-control logic.

Traction control operates by reducing the torque request when excessive wheel slip is detected.

The system is integrated with the existing torque-command path rather than directly controlling the inverter's phase current.

This allows the VCU to request a controlled reduction in drivetrain torque while leaving the inverter responsible for its normal:

* Current limits
* Torque control
* Motor control
* Ramp limiting
* Protection functions

Traction-control tuning is currently being developed through vehicle testing.

The torque reduction/ramp system uses the same **% per 10 ms** representation used by the shift-assist torque control.

### Current development status

Initial road testing has demonstrated effective traction control behaviour in second gear.

First-gear intervention is currently being tuned to reduce the aggressiveness of the initial torque cut.

---

# Gear Detection

The VCU determines the current gearbox ratio from motor speed and vehicle speed.

The calculation uses:

* Motor RPM
* Vehicle speed
* Tyre circumference
* Final drive ratio

Current drivetrain configuration:

| Parameter   |     Value |
| ----------- | --------: |
| Tyre        | 225/40R19 |
| Final drive |      4.30 |

### Gear ratios

| Gear | Ratio |
| ---: | ----: |
|    1 | 3.483 |
|    2 | 2.015 |
|    3 | 1.391 |
|    4 | 1.000 |
|    5 | 0.806 |

The detected gear is used by:

* Rev matching
* Shift validation
* Shift completion detection
* Traction-control logic
* Diagnostics

The gear calculation is deliberately independent of the dashboard RPM signal so that the drivetrain calculations can use the actual motor speed.

---

# Rev-Match Calculation

The target RPM calculation follows the drivetrain through several stages:

```text
Vehicle Speed
      │
      ▼
   Wheel RPM
      │
      ▼
Gearbox Output RPM
      │
      ▼
 Target Gear Ratio
      │
      ▼
 Base Target RPM
      │
      ▼
 Shift Gain
      │
      ▼
 Target Motor RPM
      │
      ▼
 CAN → OpenInverter
```

Because this calculation is performed continuously, the requested motor speed remains synchronised with the vehicle throughout the shift.

---

# CAN Communication

The VCU communicates with the OpenInverter controller over a **500 kbit/s CAN bus**.

## OpenInverter control frame

| Parameter  | Value         |
| ---------- | ------------- |
| CAN ID     | `0x300`       |
| Period     | 10 ms         |
| Bus speed  | 500 kbps      |
| Byte order | Little-endian |
| Payload    | 8 bytes       |

The frame contains:

* Throttle channel 1
* Throttle channel 2
* CANIO bits
* Rolling counter 1
* Cruise target RPM
* Rolling counter 2
* Regen preset
* CRC-8

### Frame layout

| Bits  | Field             |
| ----- | ----------------- |
| 0–11  | Throttle pot 1    |
| 12–23 | Throttle pot 2    |
| 24–29 | CANIO             |
| 30–31 | Rolling counter 1 |
| 32–45 | Cruise target     |
| 46–47 | Rolling counter 2 |
| 48–55 | Regen preset      |
| 56–63 | CRC               |

### CRC

```text
CRC-8
Polynomial : 0x07
Initial    : 0x00
Reflection : None
Final XOR  : None
```

Two independent 2-bit rolling counters are incremented on each transmitted frame.

The OpenInverter firmware remains responsible for the final torque/current control and associated safety limits.

---

# RX-8 Instrument Cluster

The VCU emulates the functions normally provided by the RX-8 PCM.

This allows the original dashboard to remain in the vehicle.

The system provides information including:

* Engine/motor RPM
* Vehicle speed
* Coolant temperature
* Warning lamps
* MIL
* Oil pressure
* Auxiliary status

## Dashboard CAN frame

| Parameter  | Value      |
| ---------- | ---------- |
| CAN ID     | `0x201`    |
| Period     | 20 ms      |
| Byte order | Big-endian |

Current documented fields include:

| Bytes | Data          |
| ----- | ------------- |
| 0–1   | Engine RPM    |
| 4–5   | Vehicle speed |

Dashboard RPM scaling is independent of the motor RPM used internally by the drivetrain calculations.

This allows the original RX-8 tachometer behaviour to be maintained without compromising gear detection.

---

# Configuration

Configuration parameters are stored in the Arduino's EEPROM.

Current configurable parameters include:

* Throttle calibration
* Throttle inversion
* Throttle correlation tolerance
* Upshift gain
* Downshift gain
* Torque ramp rates
* Shift timeout
* Gear confirmation timing
* Traction-control parameters

The configuration system is intended to allow drivetrain calibration without recompiling the firmware.

---

# Serial Console

A built-in serial diagnostic and tuning console is provided.

Example commands:

```text
settings
save
defaults

set upgain 1.010
set downgain 0.990
set tdiff 300

debug on
```

The console is also used during vehicle testing to monitor drivetrain behaviour and tune the control algorithms.

---

# Live Diagnostics

The diagnostic output can display live values including:

* Motor RPM
* Dashboard RPM
* Vehicle speed
* Wheel RPM
* Gearbox output RPM
* Calculated gear ratio
* Actual gear ratio
* Detected gear
* Shift state
* Rev-match target RPM
* Torque request
* Traction-control state
* CAN traffic
* Throttle values
* Digital inputs
* Inverter status

The diagnostic system is particularly useful for validating drivetrain calculations during road testing.

---

# Hardware

## Main Controller

**Arduino Nano R4**

The Nano R4 provides:

* Native CAN controller
* ADC inputs
* Digital GPIO
* Non-blocking timing
* 5 V logic

---

## CAN Transceiver

**TJA1050**

Current CAN interface:

```text
Arduino Nano R4
       │
       ▼
    TJA1050
       │
       ├── CANH
       └── CANL
```

Bus speed:

```text
500 kbps
```

A 120 Ω termination resistor can be fitted where required by the vehicle CAN topology.

---

# Power Supply

The VCU is designed for automotive 12 V electrical systems.

```text
10–20 V Automotive Supply
          │
          ▼
    TVS Protection
          │
          ▼
         Fuse
          │
          ▼
    LM2596 Buck
          │
          ▼
       5 V Rail
          │
     ┌────┼────┐
     ▼    ▼    ▼
   Nano  CAN  Inputs
```

Protection and filtering include:

* TVS diode
* Fuse
* Bulk capacitance
* Local decoupling
* 100 nF bypass capacitors

---

# Inputs

## Dual Throttle

| Input              | Pin |
| ------------------ | --- |
| Throttle channel 1 | A0  |
| Throttle channel 2 | A1  |

Typical pedal characteristics:

| Channel | Behaviour        |
| ------- | ---------------- |
| Pot 1   | 0–5 V increasing |
| Pot 2   | 5–0 V decreasing |

The Nano R4 ADC values are scaled into the 12-bit range expected by the OpenInverter interface.

```text
10-bit ADC
    │
    ▼
12-bit command
0–4095
```

The second channel is inverted in software.

Throttle plausibility and final safety handling remain part of the OpenInverter control system.

---

# Digital Inputs

The current documented inputs are:

| Pin | Function  |
| --- | --------- |
| D4  | Gear UP   |
| D5  | Gear DOWN |
| D7  | Neutral   |

Inputs use external pulldown resistors and software debounce.

---

# Auxiliary Outputs

## Oil Pump

The VCU includes an auxiliary output for oil-pump control.

```text
Output: D6
```

The output drives a logic-level N-channel MOSFET configured as a low-side switch.

Hardware includes:

* Gate resistor
* Gate pulldown
* Flyback diode

The oil-pump logic is based on engine/motor RPM and a configurable timing period.

---

# Software Architecture

The firmware is designed around a non-blocking control architecture.

Major functions include:

```cpp
updateDebounce()
calcGear()
updatePCM()
sendOpenInverterCommand()
computeCRC8()
```

The main control flow is approximately:

```text
Read Inputs
     │
     ▼
Update Vehicle State
     │
     ├── Gear Detection
     ├── Shift State Machine
     ├── Rev Match
     └── Traction Control
     │
     ▼
Update Dashboard
     │
     ▼
Generate OpenInverter Command
     │
     ▼
Transmit CAN
     │
     ▼
Process Incoming CAN
     │
     ▼
Update Auxiliary Outputs
```

Timing is non-blocking and based on the Arduino timing facilities.

---

# Control Loop Timing

| Task                     |     Period |
| ------------------------ | ---------: |
| OpenInverter control     |      10 ms |
| Dashboard CAN            |      20 ms |
| Input processing         | Continuous |
| Shift control            |      10 ms |
| Torque ramp calculations |      10 ms |

The 10 ms control period is also the basis for the torque-ramp tuning system.

Torque ramp parameters are therefore expressed as:

```text
% torque per 10 ms
```

This provides a consistent unit for both shift-assist and traction-control torque management.

---

# Responsibility Split

The VCU and OpenInverter deliberately have different responsibilities.

## VCU

The VCU handles:

* Vehicle inputs
* Throttle processing
* Gear detection
* Paddle shift logic
* Rev matching
* Traction-control torque requests
* Torque ramp generation
* CAN frame generation
* CAN CRC and rolling counters
* RX-8 dashboard emulation
* Auxiliary outputs
* EEPROM configuration
* Diagnostics

## OpenInverter

OpenInverter handles:

* Motor control
* Torque/current control
* Current limits
* Motor protection
* Inverter protection
* Final torque application
* Hardware-level control loops
* Configured inverter ramp limits

This separation keeps vehicle-level logic in the VCU while leaving the inverter responsible for the low-level motor-control and protection functions.

---

# System Architecture

```text
                    RX-8 DRIVER
                         │
          ┌──────────────┼──────────────┐
          │              │              │
      Throttle        Paddles       Vehicle
      Sensors          / Gear        Inputs
          │              │              │
          └──────────────┼──────────────┘
                         │
                         ▼
                ┌─────────────────┐
                │  Arduino Nano   │
                │      R4 VCU      │
                └─────────────────┘
                    │           │
             CAN    │           │    CAN
                    │           │
                    ▼           ▼
            OpenInverter      RX-8
             Controller     Instrument
                            Cluster
                    │
                    ▼
               Electric Motor
                    │
                    ▼
               Manual Gearbox
```

---

# Hardware Summary

| Component          | Purpose                          |
| ------------------ | -------------------------------- |
| Arduino Nano R4    | Main VCU                         |
| TJA1050            | CAN transceiver                  |
| LM2596             | Automotive supply buck converter |
| Logic-level MOSFET | Auxiliary output driver          |
| TVS diode          | Supply protection                |
| Fuse               | Supply protection                |

A dedicated PCB has been developed for the VCU and is available as a JLCPCB-manufactured board.

---

# Current Project Status

## Implemented

* [x] Arduino Nano R4 VCU
* [x] Dual-channel throttle processing
* [x] Automatic throttle calibration
* [x] Throttle plausibility checking
* [x] OpenInverter CAN communication
* [x] CRC-8 generation
* [x] Dual rolling counters
* [x] RX-8 dashboard CAN interface
* [x] Gear detection
* [x] Paddle shift state machine
* [x] Automatic shift sequencing
* [x] Continuous neutral rev matching
* [x] Configurable upshift/downshift gains
* [x] Configurable torque ramping
* [x] Torque ramp units changed to % per 10 ms
* [x] EEPROM configuration
* [x] Serial tuning console
* [x] Live diagnostics
* [x] Traction-control implementation
* [x] Initial road testing of traction control
* [x] Dedicated VCU PCB

## Currently Being Tuned

* [ ] Traction-control torque intervention
* [ ] First-gear traction-control behaviour
* [ ] Shift torque-ramp calibration
* [ ] Upshift/downshift rev-match gains
* [ ] Road testing and drivetrain calibration

The firmware is now being developed through **real vehicle testing**, with the control parameters being progressively tuned to suit the RX-8 drivetrain and the electric motor/inverter combination.

---

# Development Philosophy

The VCU is intentionally designed as a relatively simple, deterministic vehicle-control layer rather than attempting to duplicate the functions of the inverter.

The architecture is based around three principles:

### 1. Keep the inverter responsible for motor control

The VCU generates vehicle-level torque and speed requests.

The inverter remains responsible for actually controlling the motor and enforcing its own limits.

### 2. Make drivetrain behaviour tunable

Important control parameters are exposed through EEPROM configuration and the serial console so that the vehicle can be tuned without repeatedly modifying firmware.

### 3. Use real drivetrain feedback

Gear detection, rev matching and traction control are based on measured vehicle and motor behaviour rather than fixed timing assumptions.

This is particularly important for the manual gearbox, where the driver can vary the duration of a shift.

---

# Project

This project is part of the wider **electric Mazda RX-8 conversion** development.

The goal is not simply to replace the rotary engine with an electric motor, but to retain as much of the original RX-8 driving experience as possible while adding the control precision available from an electric drivetrain.

The VCU is a key part of that system, providing the bridge between the original vehicle, the manual gearbox and the OpenInverter motor controller.

---

## Status

**Active development / road testing**

The hardware and core firmware are operational. Current development is focused primarily on drivetrain calibration, particularly traction-control behaviour and torque-ramp tuning during gear changes.
