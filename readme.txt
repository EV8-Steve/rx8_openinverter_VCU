RX-8 Electric VCU Project Specification

1.  Project Overview This project implements a Vehicle Control Unit
    (VCU) using an Arduino Nano R4 to interface between:

-   Mazda RX-8 instrument cluster
-   OpenInverter running stm32-foc v5.40.R
-   Vehicle sensors and control inputs

Primary functions: - Throttle input handling - Gear-based rev-matching
shift assist - CAN control of OpenInverter - CAN emulation for RX-8
dashboard - Auxiliary output control (oil pump)

2.  Controller Hardware Main Controller: Arduino Nano R4 Features used:

-   Native CAN controller
-   5V logic and ADC range
-   10‑bit ADC
-   Digital GPIO
-   Non-blocking timers using millis()

3.  Power System Input supply: 10–20V vehicle system

Power chain: Automotive input → TVS protection → Fuse → Buck converter
(LM2596 or equivalent) → Stable 5V rail

5V rail powers: - Nano R4 - CAN transceiver - Throttle sensors - Digital
inputs

Filtering: - Bulk capacitor - 100 nF decoupling

4.  CAN Network CAN Transceiver: TJA1050

Connections: Nano D10 → TXD Nano D13 → RXD

Bus lines: CANH CANL 120Ω termination if the VCU is at a bus end

Bus speed: 500 kbps

5.  System Inputs

5.1 Throttle Pedal (Dual Channel) Analog inputs: A0 → throttle pot 1 A1
→ throttle pot 2

Characteristics: Pot1: 0‑5V increasing Pot2: 5‑0V inverted

Conversion: 10‑bit ADC scaled to 12‑bit (0–4095)

Inversion: pot2 = 4095 − (ADC × 4)

Throttle safety logic handled inside OpenInverter firmware.

5.2 Digital Inputs Each input includes a 100kΩ pulldown resistor.

Inputs: D4 → Gear UP button D5 → Gear DOWN button D7 → Gear neutral
switch

All inputs are software debounced with a 15ms delay.

6.  Outputs

Oil Pump Control Output pin: D6

Driver: Logic-level N-channel MOSFET (low-side switch)

Components: 100Ω gate resistor 100kΩ gate pulldown Flyback diode across
load

Control logic: If engineRPM ≤ 100 → start counter If counter ≥ 3000
cycles → pump OFF Else → pump ON

7.  CAN Messaging

7.1 OpenInverter Control Frame CAN ID: 0x300 Update rate: 10ms
Endianness: Little-endian

Payload layout: Bits 0–11 → throttle pot1 Bits 12–23 → throttle pot2
Bits 24–29 → CANIO bits Bits 30–31 → counter1 Bits 32–45 → cruise target
Bits 46–47 → counter2 Bits 48–55 → regen preset Bits 56–63 → CRC

CRC parameters: CRC‑8 Polynomial: 0x07 Initial value: 0x00 No reflection
No final XOR

Rolling counters: 2‑bit counters increment each frame.

7.2 RX‑8 Dashboard Message CAN ID: 0x201 Update rate: 20ms Endianness:
Big-endian

Fields: Bytes 0–1 → engine RPM Bytes 4–5 → vehicle speed

8.  Gear Detection Logic Gear estimated from engineRPM and vehicleSpeed.

Formula: ratio = (engineRPM × tyre_circumference) / (vehicleSpeed × 1667
× final_drive)

Gear ranges determine: Ratioup Ratiodown

Example gear ratios: 1 → 3.483 2 → 2.015 3 → 1.391 4 → 1.000 5 → 0.806

9.  Shift Assist System Rev‑matching shift assist.

Operation: 1. Driver presses UP or DOWN shift button 2. Current gear
ratio calculated 3. Ratio latched at shift start 4. While button is
held:

targetRPM = latched_ratio × vehicleSpeed

Vehicle speed continues updating during the shift.

5.  Button released → shift logic resets

Edge detection ensures the ratio is latched once.

10. Control Loop Timing

Tasks: OpenInverter control → 10ms Dashboard update → 20ms Input
debounce → continuous

Scheduling handled using millis().

11. Software Architecture

Main functions: updateDebounce() calcGear() updatePCM()
sendOpenInverterCommand() computeCRC8()

Loop structure: read inputs → update dash → update inverter control →
process incoming CAN → update outputs

12. Responsibilities Split

VCU (Arduino): - sensor reading - shift assist logic - CAN formatting -
dashboard emulation - auxiliary outputs

OpenInverter: - throttle safety checks - torque control - ramp limits -
current limits - motor control

13. Hardware Design Summary

Main components: Arduino Nano R4 TJA1050 CAN transceiver LM2596 DC‑DC
converter Logic MOSFET driver for outputs TVS diode for input protection

14. Key Design Features

-   deterministic CAN timing
-   gear‑ratio‑based shift assist
-   dual throttle support
-   automotive power protection
-   modular hardware architecture

15. Current Project Status System includes:

-   functional CAN messaging
-   shift assist logic
-   defined hardware architecture
-   KiCad schematic design underway
-   firmware ready for initial testing
