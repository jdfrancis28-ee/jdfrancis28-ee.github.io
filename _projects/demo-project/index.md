---
layout: post
title: Finite State Machine Keyless Entry System
description: Comparative implementation study of keyless entry systems using four distinct approaches - discrete logic, LabVIEW/myDAQ, microcontroller (embedded C), and CPLD (Verilog/VHDL) - with analysis of cost, power, size, and flexibility trade-offs.
skills: 
  - Digital Logic Design
  - Finite State Machines
  - Embedded C Programming
  - Verilog/VHDL
  - LabVIEW
  - Microcontroller Programming (dsPIC33EP)
  - CPLD Development
  - Comparative Engineering Analysis
  - System Timing Validation
main-image: /assets/images/projects/fsm-keyless-entry.jpg
---

## Project Overview

This project explores four fundamentally different approaches to implementing a keyless entry system, demonstrating the engineering trade-offs between hardware simplicity, flexibility, cost, and power consumption. Each implementation meets identical functional requirements while utilizing vastly different underlying technologies.

**Design Requirement:** Secure 4-digit keyless entry system with lockout after 3 failed attempts

**Four Implementations:**
1. Discrete Logic (NAND gates + D flip-flops)
2. LabVIEW with myDAQ hardware
3. Microcontroller (dsPIC33EP with embedded C)
4. CPLD (Complex Programmable Logic Device with Verilog/VHDL)

---

## Implementation 1: Discrete Logic

### Design Approach
Pure combinational and sequential logic using 7400-series NAND gates and D flip-flops.

**Key Components:**
- 74LS00 Quad 2-input NAND gates
- 74LS74 Dual D flip-flops
- Discrete state encoding using flip-flops

**Advantages:**
- Lowest component cost (~$5)
- Fastest response time (<10ns propagation delay)
- No software dependencies
- Simple, deterministic behavior

**Disadvantages:**
- Zero flexibility (cannot modify code without rewiring)
- Large physical footprint
- Difficult to debug
- Complex wiring prone to errors

### Circuit Implementation
[Add circuit schematic image]

{% include image-gallery.html images="/assets/images/projects/discrete-logic-schematic.jpg" height="400" %}

---

## Implementation 2: LabVIEW/myDAQ

### Design Approach
Graphical programming environment with National Instruments myDAQ data acquisition hardware.

**Key Features:**
- Visual state machine programming
- Real-time monitoring and debugging
- Built-in oscilloscope and function generator
- Direct hardware I/O control

**Advantages:**
- Rapid prototyping and testing
- Excellent debugging capabilities
- Educational value (visual programming)
- Easy to modify logic

**Disadvantages:**
- Requires PC connection (not standalone)
- Higher cost (~$200 for myDAQ)
- Slower response time (~1ms)
- Power consumption tied to PC

### LabVIEW Block Diagram
[Add LabVIEW VI screenshot]

{% include image-gallery.html images="/assets/images/projects/labview-block-diagram.jpg" height="400" %}

---

## Implementation 3: Microcontroller (dsPIC33EP + Embedded C)

### Design Approach
Embedded C programming on Microchip dsPIC33EP digital signal controller.

**Key Features:**
- C-based state machine implementation
- Hardware debouncing via software
- Low-power sleep modes
- UART debugging interface

**Code Snippet:**
```c
typedef enum {
    STATE_IDLE,
    STATE_DIGIT_1,
    STATE_DIGIT_2,
    STATE_DIGIT_3,
    STATE_DIGIT_4,
    STATE_UNLOCK,
    STATE_LOCKOUT
} KeypadState;

void processInput(uint8_t keyPressed) {
    switch(currentState) {
        case STATE_IDLE:
            if(keyPressed == correctCode[0]) {
                currentState = STATE_DIGIT_1;
                resetFailureCount();
            } else {
                incrementFailureCount();
            }
            break;
        // ... additional states
    }
}
```

**Advantages:**
- Highly flexible (reprogram via USB)
- Low power consumption (<10mA active, <1µA sleep)
- Small footprint (28-pin DIP package)
- Rich peripheral set (timers, PWM, ADC)

**Disadvantages:**
- Requires programming toolchain
- Higher complexity than discrete logic
- Response time slower than hardware (~100µs)

---

## Implementation 4: CPLD (Verilog/VHDL)

### Design Approach
Hardware description language (HDL) implementation on Lattice iCE40 CPLD.

**Verilog State Machine:**
```verilog
module keyless_entry (
    input clk,
    input reset,
    input [3:0] key_input,
    output reg unlock,
    output reg lockout
);

    // State encoding
    parameter IDLE     = 3'd0;
    parameter DIGIT_1  = 3'd1;
    parameter DIGIT_2  = 3'd2;
    parameter DIGIT_3  = 3'd3;
    parameter DIGIT_4  = 3'd4;
    parameter UNLOCK   = 3'd5;
    parameter LOCKOUT  = 3'd6;
    
    reg [2:0] state, next_state;
    reg [1:0] fail_count;
    
    // State machine logic
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            fail_count <= 2'd0;
        end else begin
            state <= next_state;
        end
    end
    
    // Next state and output logic
    always @(*) begin
        case(state)
            IDLE: begin
                if(key_input == 4'd1) next_state = DIGIT_1;
                else begin
                    fail_count = fail_count + 1;
                    if(fail_count >= 3) next_state = LOCKOUT;
                end
            end
            // ... additional states
        endcase
    end
    
endmodule
```

**Advantages:**
- Hardware-speed execution (parallel logic)
- Reconfigurable (reprogram CPLD)
- Very low power (<5mA)
- Smallest physical size

**Disadvantages:**
- Highest initial cost (~$50-100 dev board)
- Requires HDL knowledge
- More complex debugging than software

---

## Comparative Analysis

### Performance Metrics

| Metric | Discrete Logic | LabVIEW | Microcontroller | CPLD |
|--------|---------------|---------|-----------------|------|
| **Response Time** | <10 ns | ~1 ms | ~100 µs | <50 ns |
| **Power (Active)** | 50 mW | 15 W (PC) | 30 mW | 15 mW |
| **Power (Idle)** | 50 mW | N/A | <1 mW | 1 mW |
| **Component Cost** | $5 | $200 | $15 | $75 |
| **Physical Size** | Large (breadboard) | Medium (myDAQ) | Small (DIP28) | Smallest (QFN) |
| **Flexibility** | None | Highest | High | High |
| **Debugging** | Difficult | Excellent | Good | Moderate |

### Design Trade-Off Analysis

**Cost vs. Flexibility:**
Discrete logic offers lowest cost but zero flexibility post-implementation. CPLD and microcontroller provide reconfigurability at moderate cost increase.

**Speed vs. Power:**
Discrete logic and CPLD achieve fastest response times but at cost of higher static power consumption. Microcontroller offers best power efficiency with sleep modes.

**Development Time vs. Production Cost:**
LabVIEW enables fastest prototyping but unsuitable for production. Microcontroller strikes best balance for low-volume production.

---

## Testing & Validation

### Functional Testing
All four implementations successfully validated against identical test cases:
- Correct 4-digit code entry → Unlock
- Incorrect entry → Remain locked
- 3 consecutive failures → Lockout mode
- Reset functionality

### Timing Analysis
Measured response times using oscilloscope for hardware implementations and UART timestamps for software implementations.

{% include image-gallery.html images="/assets/images/projects/timing-measurements.jpg" height="400" %}

---

## Lessons Learned

**Design Methodology:**
- No single "best" implementation - optimal choice depends on application requirements
- Cost-optimized solutions (discrete logic) sacrifice flexibility
- Development tools significantly impact prototyping speed

**Technical Skills:**
- Hands-on experience with HDL synthesis and CPLD programming
- Embedded C state machine patterns
- Hardware/software co-design trade-offs

**Engineering Trade-offs:**
- Performance vs. power consumption
- Development time vs. unit cost
- Flexibility vs. simplicity

---

## Future Improvements

**Potential Enhancements:**
- Add biometric authentication (fingerprint sensor)
- Implement encrypted communication for remote unlock
- Battery-powered operation with energy harvesting
- Multi-user support with individual access codes

**Additional Implementations:**
- FPGA implementation (Xilinx Artix-7)
- Arduino-based version for cost comparison
- Raspberry Pi implementation with GUI

---

## Project Files

**GitHub Repository:** [Link to your repo]

**Files Included:**
- Discrete logic schematics (KiCad format)
- LabVIEW VI files
- Embedded C source code (MPLAB X project)
- Verilog HDL source
- Test vectors and validation scripts
- Bill of materials (BOM) for each implementation

---

## Conclusion

This comparative study demonstrates that engineering design requires careful analysis of application requirements before selecting an implementation approach. While discrete logic offers unbeatable speed and simplicity, modern programmable solutions (microcontroller, CPLD) provide flexibility that outweighs their modest cost premium for most applications.

The microcontroller implementation emerged as the most practical choice for real-world deployment, balancing cost, power, flexibility, and development time.

---

*Project completed as part of EE 200 Design Tools at Penn State University, Spring 2024*


