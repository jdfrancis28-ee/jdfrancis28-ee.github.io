---
layout: post
title: Finite State Machine Keyless Entry System
description: Comparative implementation study of automotive keyless entry system using four distinct approaches - discrete logic (NAND gates), LabVIEW/myDAQ, microcontroller (embedded C), and CPLD (WinCUPL) - demonstrating engineering tradeoffs in cost, power, size, and flexibility.
skills: 
  - Digital Logic Design
  - Finite State Machines
  - Embedded C Programming
  - Verilog/VHDL
  - WinCUPL (CPLD Programming)
  - LabVIEW
  - Microcontroller Programming (dsPIC33EP)
  - Comparative Engineering Analysis
  - System-Level Design
  - Hardware Description Languages
main-image: /assets/projects/fsm-keyless-entry.png
---

## Project Overview

This project implements an automotive-style keyless entry system using four fundamentally different technologies, demonstrating the critical engineering tradeoffs between hardware simplicity, software flexibility, cost, power consumption, and development time.

**Design Challenge:** Build the same keyless entry state machine four different ways to understand when to use each technology.

**Four Implementations:**
1. **Discrete Logic** - NAND gates and D flip-flops (pure hardware)
2. **LabVIEW/myDAQ** - Graphical programming with data acquisition hardware
3. **Microcontroller** - dsPIC33EP with embedded C firmware
4. **CPLD** - Atmel ATF750C with WinCUPL HDL

---

## System Specifications

### Inputs & Outputs

**Inputs:**
- **Lock (L):** Button to lock all doors (active low)
- **Unlock (U):** Button to unlock doors (active low)
- **Side Door (S):** Sensor detecting door open/close (active low)

**Outputs:**
- **Driver Door (DD):** Indicator for driver's door unlocked
- **All Doors (AD):** Indicator for all doors unlocked
- **Side Door Open (SD):** Indicator for side door currently open

### State Machine Logic

**State 0 (S0 - All Locked):** All doors locked → DD=0, AD=0, SD=0

**State 1 (S1 - Driver Unlocked):** First unlock press unlocks driver's door → DD=1, AD=0, SD=0

**State 2 (S2 - All Unlocked):** Second unlock press (within 3-second timeout) unlocks all doors → DD=1, AD=1, SD=0

**State 3 (S3 - Side Door Open):** Side door sensor activates when door opens → DD=1, AD=1, SD=1

**Lock button:** Returns to State 0 from any state

---

## State Diagram




