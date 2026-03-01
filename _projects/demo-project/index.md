---
layout: post
title: Finite State Machine Keyless Entry System
description: Comparative implementation study of an automotive keyless entry system using four distinct approaches - discrete logic (NAND gates), LabVIEW/myDAQ, microcontroller (embedded C), and CPLD (WinCUPL) - demonstrating engineering tradeoffs in cost, power, size, and flexibility.
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
main-image: /assets/projects/discrete-logic-circuit.png
---

## Project Overview

This project implements an automotive keyless entry system four different ways — discrete logic, LabVIEW, embedded C on a microcontroller, and a CPLD — to study the engineering tradeoffs between each approach. The goal was not just to make the system work, but to understand when each technology is the right tool for the job.

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

**State 0 (S0 - All Locked):** All doors locked. DD=0, AD=0, SD=0

**State 1 (S1 - Driver Unlocked):** First unlock press unlocks driver's door. DD=1, AD=0, SD=0

**State 2 (S2 - All Unlocked):** Second unlock press within 3-second timeout unlocks all doors. DD=1, AD=1, SD=0

**State 3 (S3 - Side Door Open):** Side door sensor activates when door opens. DD=1, AD=1, SD=1

**Lock button:** Returns to State 0 from any state

---

## State Diagram

```
                    ┌─────────────┐
             ┌──────┤   State 0   │◄──────┐
             │      │ All Locked  │       │ Lock (L)
             │      │ DD=0 AD=0   │       │ pressed
             │      └──────┬──────┘       │
             │             │              │
             │        Unlock (U)          │
             │        pressed             │
             │             │              │
             │             ▼              │
             │      ┌─────────────┐       │
             │      │   State 1   │───────┤
             │      │Driver Unlock│       │
             │      │ DD=1 AD=0   │       │
             │      └──────┬──────┘       │
             │             │              │
             │        Unlock (U)          │
             │        pressed again       │
             │      (within 3s timeout)   │
             │             │              │
             │             ▼              │
             │      ┌─────────────┐       │
             │      │   State 2   │───────┤
             │      │ All Unlocked│       │
             │      │ DD=1 AD=1   │       │
             │      └──────┬──────┘       │
             │             │              │
             │        Side Door (S)       │
             │        opens               │
             │             │              │
             │             ▼              │
             │      ┌─────────────┐       │
             └─────▶│   State 3   │───────┘
                    │Side Door Opn│
                    │  DD=1 AD=1  │
                    │    SD=1     │
                    └─────────────┘
```

---

## Implementation 1: Discrete Logic (NAND Gates + D Flip-Flops)

The simplest approach at the hardware level. The state machine is built entirely from physical gates and flip-flops with no software involved, making it the fastest implementation but completely inflexible once wired.

### Hardware Components

- **74HC00** Quad 2-input NAND gates (for next-state logic)
- **74HC74** Dual D flip-flops (for state register)
- Discrete resistors, LEDs, pushbuttons

### Truth Table for Next-State Logic

| Current State | Lock | Unlock | Side Door | Next State | Implementation |
|---------------|------|--------|-----------|------------|----------------|
| S0 (00)       | X    | 1      | X         | S1 (01)    | NAND logic for D1, D0 inputs |
| S1 (01)       | 1    | X      | X         | S0 (00)    | Reset flip-flops |
| S1 (01)       | 0    | 1      | X         | S2 (10)    | Set D1, clear D0 |
| S2 (10)       | 0    | X      | 1         | S3 (11)    | Set both flip-flops |

### Circuit Schematic

![Discrete Logic Circuit](/assets/projects/discrete-logic-circuit.png)

### Advantages

✅ **Fastest response** - Pure hardware, <10ns propagation delay  
✅ **Deterministic** - No software timing issues  
✅ **Educational** - Teaches fundamental digital logic  
✅ **No programming** - Just wire connections  

### Disadvantages

❌ **Zero flexibility** - Any change requires complete rewiring  
❌ **Large footprint** - Multiple ICs on breadboard  
❌ **Difficult debugging** - Must probe signals with oscilloscope  
❌ **Error-prone wiring** - Easy to make connection mistakes  

### Performance Metrics

| Metric | Value |
|--------|-------|
| Response Time | <10 ns |
| Power Consumption | ~100 mW |
| Component Count | 5-7 ICs + passives |
| Cost | $10-15 |
| Development Time | 2-3 days |

---

## Implementation 2: LabVIEW/myDAQ

LabVIEW allowed the state machine to be built graphically using drag-and-drop programming, making it the fastest to prototype and easiest to debug. However it requires a PC to run, making it unsuitable for any real deployment.

### LabVIEW VI Structure

**State Machine Loop:**
- Event structure monitors button presses
- Case structure implements state transitions
- Shift registers maintain current state
- Timeout timer manages 3-second unlock window

### Block Diagram

![LabVIEW Block Diagram](/assets/projects/labview-block-diagram.png)

### Key LabVIEW Features Used

**Digital I/O:**
- `DAQmx Create Channel` - Configure DIO lines
- `DAQmx Read` - Read button states
- `DAQmx Write` - Control LED outputs

**State Machine:**
- Enum for state encoding (S0, S1, S2, S3)
- Case structure for state-specific logic
- Event structure for button press handling

### Code Snippet (Pseudo-LabVIEW)

```
Initialize:
  - Set state = S0
  - Configure myDAQ DIO channels
  - Start timeout timer

Main Loop:
  Case (current_state):
    S0: If unlock_pressed → state = S1, start 3s timer
    S1: If lock_pressed → state = S0
        If unlock_pressed AND timer_active → state = S2
        If timer_expired → state = S0
    S2: If lock_pressed → state = S0
        If side_door_open → state = S3
    S3: If lock_pressed → state = S0
        If side_door_closed → state = S2
  
  Update LED outputs based on state
```

### Advantages

✅ **Rapid prototyping** - Drag-and-drop programming  
✅ **Excellent debugging** - Real-time probes and indicators  
✅ **Easy modification** - Change logic without rewiring  
✅ **Built-in visualization** - See state transitions in real-time  

### Disadvantages

❌ **Not standalone** - Requires PC connection  
❌ **High cost** - LabVIEW license + myDAQ (~$200-300)  
❌ **Not production-viable** - Cannot deploy to embedded system  
❌ **Power hungry** - PC must remain powered on  

### Performance Metrics

| Metric | Value |
|--------|-------|
| Response Time | ~1 ms (software loop) |
| Power Consumption | High (PC-dependent) |
| Component Count | myDAQ + PC |
| Cost | $200-300 |
| Development Time | 4-6 hours |

---

## Implementation 3: Microcontroller (dsPIC33EP + Embedded C)

The most practical implementation for a real product. The dsPIC33EP runs the state machine in firmware, with a 16x2 LCD display showing the current state in real time for feedback and debugging. A single low-cost chip handles all inputs, outputs, timing, and display logic.

### Hardware Platform

**Microcontroller:** Microchip dsPIC33EP64MC502 (28-pin DIP)
- 16-bit DSC with 64KB flash, 16KB RAM
- Operating at 3.685 MHz (internal FRC oscillator with PLL)
- Interrupt-driven architecture for responsive button handling

**Peripherals:**
- HD44780-compatible 16x2 LCD display showing current state in real time
- 3x pushbuttons (Lock, Unlock, Side Door simulation)
- 3x status LEDs (DD, AD, SD outputs)

### Pin Connections

| Pin  | Function | Connection |
|------|----------|------------|
| RB8  | Lock Button | Active low, internal pull-up enabled |
| RB9  | Unlock Button (INT1) | Active low, interrupt-driven |
| RB10 | Side Door Sensor | Active low, polled in main loop |
| RB11 | Driver Door LED (DD) | Active high output |
| RB12 | All Doors LED (AD) | Active high output |
| RB13 | Side Door LED (SD) | Active high output |
| RB14-RB15 | LCD Data (D4-D5) | 4-bit mode |
| RA0  | LCD RS | Register select |
| RA1  | LCD EN | Enable strobe |

### Software Architecture

**State Machine Implementation:**

```c
typedef enum {
    STATE_ALL_LOCKED = 0,      // S0: DD=0, AD=0, SD=0
    STATE_DRIVER_UNLOCKED = 1, // S1: DD=1, AD=0, SD=0
    STATE_ALL_UNLOCKED = 2,    // S2: DD=1, AD=1, SD=0
    STATE_SIDE_DOOR_OPEN = 3   // S3: DD=1, AD=1, SD=1
} SystemState;

volatile SystemState currentState = STATE_ALL_LOCKED;
volatile unsigned int unlockTimeout = 0;  // Timer ticks (10ms each)
```

**Main Program Flow:**

```c
int main(void) {
    initialize_IO_ports();       // Configure pins, pull-ups
    initialize_timer();          // Timer1: 10ms periodic interrupt
    init_LCD_module();           // LCD initialization sequence
    initialize_interrupts();     // INT1 for unlock button
    
    currentState = STATE_ALL_LOCKED;
    update_outputs();            // Set initial LED states
    display_state(currentState); // LCD shows "State: S0 All Locked"
    
    while(1) {
        process_buttons();       // Poll lock button & side door
        __delay_ms(50);          // 50ms debounce delay
    }
}
```

**Interrupt Service Routine (Unlock Button):**

```c
void __attribute__((__interrupt__, auto_psv)) _INT1Interrupt(void) {
    if(debounceCounter == 0) {
        debounceCounter = 5;  // 50ms debounce
        
        switch(currentState) {
            case STATE_ALL_LOCKED:
                currentState = STATE_DRIVER_UNLOCKED;
                unlockTimeout = 300;  // 3 seconds (300 x 10ms)
                break;
                
            case STATE_DRIVER_UNLOCKED:
                if(unlockTimeout > 0) {  // Within timeout window
                    currentState = STATE_ALL_UNLOCKED;
                    unlockTimeout = 0;
                }
                break;
        }
        
        update_outputs();
        display_state(currentState);
    }
    
    IFS1bits.INT1IF = 0;  // Clear interrupt flag
}
```

**Timer ISR (Timeout Management):**

```c
void __attribute__((__interrupt__, auto_psv)) _T1Interrupt(void) {
    if(unlockTimeout > 0) {
        unlockTimeout--;
        
        // Timeout expired in State 1 - return to locked
        if(unlockTimeout == 0 && currentState == STATE_DRIVER_UNLOCKED) {
            currentState = STATE_ALL_LOCKED;
            update_outputs();
            display_state(currentState);
        }
    }
    
    IFS0bits.T1IF = 0;
}
```

**Output Control:**

```c
void update_outputs(void) {
    switch(currentState) {
        case STATE_ALL_LOCKED:
            DRIVER_DOOR_LED = 0;  // DD = 0
            ALL_DOORS_LED = 0;    // AD = 0
            SIDE_DOOR_LED = 0;    // SD = 0
            break;
            
        case STATE_DRIVER_UNLOCKED:
            DRIVER_DOOR_LED = 1;  // DD = 1
            ALL_DOORS_LED = 0;    // AD = 0
            SIDE_DOOR_LED = 0;    // SD = 0
            break;
            
        case STATE_ALL_UNLOCKED:
            DRIVER_DOOR_LED = 1;  // DD = 1
            ALL_DOORS_LED = 1;    // AD = 1
            SIDE_DOOR_LED = 0;    // SD = 0
            break;
            
        case STATE_SIDE_DOOR_OPEN:
            DRIVER_DOOR_LED = 1;  // DD = 1
            ALL_DOORS_LED = 1;    // AD = 1
            SIDE_DOOR_LED = 1;    // SD = 1
            break;
    }
}
```

### Circuit Schematic

![Microcontroller Circuit](/assets/projects/microcontroller-circuit.png)

### Circuit Realization

![Microcontroller Circuit](/assets/projects/CircuitFSM.png)

### Advantages

✅ **Most flexible** - Easy firmware updates via USB programmer  
✅ **Lowest cost** - $5-10 for complete solution  
✅ **Smallest footprint** - Single 28-pin chip  
✅ **Real-time feedback** - LCD displays current state during operation  
✅ **Low power** - Sleep modes available (<1mW idle)  

### Disadvantages

❌ **Requires programming** - C knowledge necessary  
❌ **Software bugs possible** - Unlike pure hardware  
❌ **Slower response** - ~100us vs <10ns for discrete logic  
❌ **Development tools** - Need MPLAB X IDE and PICkit programmer  

### Performance Metrics

| Metric | Value |
|--------|-------|
| Response Time | ~100 us |
| Power Consumption | 10-30 mW active, <1 mW sleep |
| Component Count | 1 IC + LCD + passives |
| Cost | $5-10 |
| Development Time | 1-2 days |

---

## Implementation 4: CPLD (ATF750C + WinCUPL)

The CPLD bridges the gap between discrete logic and a microcontroller — reconfigurable like software but executing at hardware speed. State transitions are described in WinCUPL and compiled directly into the chip's logic fabric.

### Hardware Platform

**CPLD:** Atmel ATF750C-10JC (44-pin PLCC)
- 128 macrocells, 10ns propagation delay
- In-system programmable via JTAG

**External Timer:** CD4541BE CMOS Programmable Timer
- Provides 3-second timeout for unlock sequence
- RC time constant: R=1MOhm, C=3.3uF

### WinCUPL State Machine Code

```cupl
Name     Project_4;
Date     5/2/2024;
Designer Jacob Francis;
Company  EE 200;
Device   v750c;

/* INPUT PINS */
PIN 1  = CLK;   /* Clock Input */
PIN 2  = !L;    /* Lock Button (active low) */
PIN 3  = !U;    /* Unlock Button (active low) */
PIN 4  = !S;    /* Side Door Sensor (active low) */
PIN 5  = TO;    /* Timeout signal from CD4541BE */

/* OUTPUT PINS */
PIN 23 = DD;    /* Driver Door Unlocked */
PIN 22 = AD;    /* All Doors Unlocked */
PIN 21 = SD;    /* Side Door Open */
PIN 20 = TMR;   /* Timer Enable (to CD4541BE !MR pin) */

/* STATE REGISTER */
FIELD state = [DD, AD, SD];

/* STATE DEFINITIONS */
$define S0  'b'000    /* All Locked:      DD=0 AD=0 SD=0 */
$define S1  'b'100    /* Driver Unlocked: DD=1 AD=0 SD=0 */
$define S2  'b'110    /* All Unlocked:    DD=1 AD=1 SD=0 */
$define S3  'b'111    /* Side Door Open:  DD=1 AD=1 SD=1 */

/* STATE MACHINE */
SEQUENCE state {
    
    PRESENT S0
        IF U NEXT S1;       /* Unlock pressed - Driver unlocked */
        DEFAULT NEXT S0;
    
    PRESENT S1
        IF L NEXT S0;       /* Lock pressed - All locked */
        IF U & !TO NEXT S2; /* Unlock within timeout - All unlocked */
        IF TO NEXT S0;      /* Timeout expired - All locked */
        DEFAULT NEXT S1;
    
    PRESENT S2
        IF L NEXT S0;       /* Lock pressed - All locked */
        IF S NEXT S3;       /* Side door opened */
        DEFAULT NEXT S2;
    
    PRESENT S3
        IF L NEXT S0;       /* Lock pressed - All locked */
        IF !S NEXT S2;      /* Side door closed - All unlocked */
        DEFAULT NEXT S3;
}

/* TIMER CONTROL OUTPUT */
/* Timer enabled only in State S1 */
TMR = (state:S1);
```

### CD4541BE Timer Configuration

**Pin Connections:**
- Pin 3 (CLK): RC oscillator (R=1MOhm, C=3.3uF, ~3.3s timeout)
- Pin 5 (!MR): Connected to CPLD TMR output (active low reset)
- Pin 7 (Q): Timeout output to CPLD TO input
- Pin 10 (AUTO/!MAN): Tied to GND (auto-trigger mode)

**Operation:**
- When TMR=LOW (State S1): Timer running, Q goes HIGH after 3.3s
- When TMR=HIGH (other states): Timer disabled, Q=LOW
- Creates a 3-second window for the second unlock press

### Circuit Schematic

![CPLD Circuit](/assets/projects/cpld-circuit.png)

### Advantages

✅ **Hardware-speed execution** - <50ns state transitions  
✅ **Reconfigurable** - Reprogram CPLD in-circuit via JTAG  
✅ **Compact** - More integrated than discrete logic  
✅ **Deterministic timing** - No software jitter  
✅ **Low power** - CMOS technology (~20mW)  

### Disadvantages

❌ **Learning curve** - WinCUPL Boolean equation syntax  
❌ **Programmer required** - Need Atmel-ICE or equivalent  
❌ **Limited I/O** - Fewer pins than microcontroller  
❌ **External timer** - CD4541BE adds complexity for long delays  

### Performance Metrics

| Metric | Value |
|--------|-------|
| Response Time | <50 ns |
| Power Consumption | ~20 mW |
| Component Count | 2 ICs (CPLD + timer) |
| Cost | $15-25 |
| Development Time | 1-2 days |

---

## Comparative Analysis

### Performance Comparison Table

| Criteria | Discrete Logic | LabVIEW/myDAQ | Microcontroller | CPLD |
|----------|---------------|---------------|-----------------|------|
| **Response Time** | <10 ns | ~1 ms | ~100 us | <50 ns |
| **Power (Active)** | 100 mW | High (PC) | 30 mW | 20 mW |
| **Power (Sleep)** | 100 mW | N/A | <1 mW | 20 mW |
| **Component Cost** | $10-15 | $200-300 | $5-10 | $15-25 |
| **Physical Size** | Large | Medium | Very Small | Small |
| **Flexibility** | None | Highest | Very High | High |
| **Debugging** | Difficult | Excellent | Good | Moderate |
| **Development Time** | 2-3 days | 4-6 hours | 1-2 days | 1-2 days |
| **Production Viability** | Poor | None | **Excellent** | Good |

### Engineering Tradeoffs

**Cost vs. Flexibility:**
Discrete logic offers the lowest initial cost but zero flexibility post-implementation. The microcontroller adds roughly $5 but allows unlimited firmware updates without any hardware changes.

**Speed vs. Power:**
Discrete logic and the CPLD achieve the fastest response times but consume more static power. The microcontroller offers the best power efficiency through hardware sleep modes.

**Development Time vs. Complexity:**
LabVIEW enables the fastest initial prototype but cannot be deployed to a standalone system. The microcontroller balances development speed with real-world deployment viability.

---

## Lessons Learned

**State Machine Design:**
Implementing the same specification four different ways made it clear how much the underlying technology shapes the design approach. State encoding, timing management, and debugging all look completely different depending on whether you are working in hardware, firmware, or graphical tools.

**Choosing the Right Tool:**
There is no universal best implementation. The right choice depends on what the application actually requires:

- **Production deployment** — Microcontroller (cost, flexibility, size)
- **Absolute fastest response** — Discrete logic or CPLD (hardware speed)
- **Rapid prototyping** — LabVIEW (visual programming, built-in debugging)
- **Learning fundamentals** — Discrete logic (teaches Boolean logic from the ground up)
- **Field reconfigurability** — CPLD or microcontroller

Real engineering means matching the technology to the requirements, not defaulting to whatever is most familiar.

---

## Conclusion

This project demonstrated that the same functional specification can be realized in fundamentally different ways, each with distinct tradeoffs in speed, cost, flexibility, and development effort. While discrete logic provides unmatched response time and LabVIEW excels at rapid prototyping, the microcontroller proved to be the most practical solution for production deployment — balancing low cost, small footprint, firmware flexibility, and real-time LCD feedback.

The core takeaway is that understanding when to use each technology is just as important as knowing how to use it.
