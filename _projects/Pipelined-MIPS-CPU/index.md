---
layout: post
title: 5-Stage Pipelined MIPS CPU on FPGA
description: Complete implementation of a pipelined processor based on MIPS architecture with hazard detection, data forwarding, and pipeline stalling. Designed in Verilog HDL and successfully synthesized and deployed to Xilinx XC7Z010 FPGA hardware.
skills: 
  - Verilog HDL
  - FPGA Development
  - Computer Architecture
  - Pipeline Optimization
  - Hazard Detection & Resolution
  - Digital System Design
  - Xilinx Vivado
  - Hardware Verification
  - RTL Design
main-image: /assets/images/projects/fpga-pipelined-cpu.jpg
---

## Project Overview

This project implements a complete 5-stage pipelined CPU based on the MIPS instruction set architecture, demonstrating fundamental concepts of modern processor design including instruction-level parallelism, hazard detection, and data forwarding mechanisms.

**Design Goal:** Build a functional pipelined processor that executes MIPS instructions with minimal stalls

**Key Features:**
- Classic 5-stage pipeline (IF, ID, EX, MEM, WB)
- Data forwarding to minimize pipeline stalls
- Load-use hazard detection and stalling
- Separate instruction and data memories
- Successfully deployed to Xilinx FPGA

---

## Pipeline Architecture

### The Five Stages

**1. Instruction Fetch (IF)**
- Program Counter (PC) maintains current instruction address
- Instruction memory fetches 32-bit instruction
- PC incremented by 4 for next instruction

**2. Instruction Decode (ID)**
- Control Unit decodes opcode and generates control signals
- Register File reads source operands (rs, rt)
- Immediate values sign-extended to 32 bits
- Hazard detection logic checks for dependencies

**3. Execute (EX)**
- Arithmetic Logic Unit (ALU) performs operations
- Forwarding multiplexers select correct operand sources
- Supports: ADD, SUB, AND, OR, XOR operations

**4. Memory Access (MEM)**
- Data memory read/write for load/store instructions
- Pass-through stage for arithmetic operations

**5. Write Back (WB)**
- Result written back to Register File
- Data source selected from ALU result or memory

### Pipeline Diagram

```
┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐
│   IF    │───▶│   ID    │───▶│   EX    │───▶│   MEM   │───▶│   WB    │
│ Fetch   │    │ Decode  │    │ Execute │    │ Memory  │    │ Write   │
└─────────┘    └─────────┘    └─────────┘    └─────────┘    └─────────┘
     │              │              │              │              │
     └──────────────┴──────────────┴──────────────┴──────────────┘
                         Forwarding Paths
```

---

## Hazard Handling Implementation

### Data Hazards

**Problem:** Subsequent instructions may need data before it's been written back to the register file.

**Example Hazard:**
```assembly
add $3, $1, $2    # $3 = $1 + $2 (result ready in EX stage)
sub $4, $3, $5    # Needs $3 immediately (not yet written back!)
```

### Solution 1: Data Forwarding

Route data directly from later pipeline stages to earlier stages, bypassing the register file.

**Forwarding Logic (Verilog):**
```verilog
// EX Hazard Detection - Forward from EX/MEM pipeline register
if ((ewreg == 1'b1) && (edestReg != 5'b0) && 
    (edestReg == rs) && (em2reg == 1'b0)) begin
    fwda = 2'b01;  // Forward ALU result from EX stage
end

// MEM Hazard Detection - Forward from MEM/WB pipeline register
else if ((mwreg == 1'b1) && (mdestReg != 5'b0) && 
         (mdestReg == rs) && (mm2reg == 1'b0)) begin
    fwda = 2'b10;  // Forward ALU result from MEM stage
end
```

**Forwarding Multiplexer:**
```verilog
case (fwd)
    2'b00: q = regOut;  // No hazard - use register file data
    2'b01: q = r;       // Forward from EX/MEM (ALU result)
    2'b10: q = mr;      // Forward from MEM/WB (ALU result)
    2'b11: q = mdo;     // Forward from MEM/WB (memory data)
endcase
```

**Result:** Eliminates most data hazards without pipeline stalls

---

### Solution 2: Pipeline Stalling

For load-use hazards, forwarding isn't sufficient because memory data arrives too late. The pipeline must stall for one cycle.

**Load-Use Hazard Example:**
```assembly
lw  $3, 0($1)     # Load $3 from memory (data ready in MEM stage)
add $4, $3, $5    # Needs $3 in EX stage (can't forward in time!)
```

**Stalling Logic (Verilog):**
```verilog
// Detect load-use hazard
if ((ewreg == 1'b1) && (em2reg == 1'b1) && (edestReg != 5'b0) &&
    ((edestReg == rs) || (edestReg == rt))) begin
    // Insert pipeline bubble
    wreg = 1'b0;   // Convert current instruction to NOP
    wmem = 1'b0;
    wpcir = 1'b0;  // Stall PC and IF/ID register
end
```

**Performance Impact:** 1-cycle stall penalty per load-use hazard

---

### Structural Hazards

**Problem:** Multiple instructions trying to access the same hardware resource simultaneously.

**Solution:** Separate instruction and data memories
- IF stage uses instruction memory
- MEM stage uses data memory
- No resource conflicts possible

---

## Control Unit Design

The Control Unit is the most complex module, responsible for:
- Instruction decoding
- Control signal generation
- Hazard detection
- Forwarding control

### Control Signals Generated

| Signal | Purpose | Values |
|--------|---------|--------|
| `wreg` | Write register enable | 1=write, 0=no write |
| `m2reg` | Memory to register | 1=load instruction |
| `wmem` | Write memory enable | 1=store instruction |
| `aluc[3:0]` | ALU operation code | 0010=ADD, 0110=SUB, etc. |
| `aluimm` | ALU immediate operand | 1=use immediate, 0=use register |
| `regrt` | Destination register | 1=rt (I-type), 0=rd (R-type) |
| `fwda[1:0]` | Forward control for rs | See forwarding table |
| `fwdb[1:0]` | Forward control for rt | See forwarding table |
| `wpcir` | Pipeline advance control | 0=stall, 1=advance |

### Control Unit State Machine

```verilog
always @(*) begin
    // Default control signals
    wreg = 1'b0;
    m2reg = 1'b0;
    wmem = 1'b0;
    wpcir = 1'b1;  // Default: advance pipeline
    
    // Decode instruction opcode
    case (op)
        6'b000000: begin  // R-type instructions
            wreg = 1'b1;
            regrt = 1'b0;  // Write to rd
            
            case (func)
                6'b100000: aluc = 4'b0010;  // ADD
                6'b100010: aluc = 4'b0110;  // SUB
                6'b100100: aluc = 4'b0000;  // AND
                6'b100101: aluc = 4'b0001;  // OR
                6'b100110: aluc = 4'b1110;  // XOR
            endcase
        end
        
        6'b100011: begin  // LW (Load Word)
            wreg = 1'b1;
            m2reg = 1'b1;
            aluimm = 1'b1;
            regrt = 1'b1;  // Write to rt
        end
    endcase
    
    // Hazard detection and stalling
    if ((ewreg == 1'b1) && (em2reg == 1'b1) && 
        (edestReg != 5'b0) && ((edestReg == rs) || (edestReg == rt))) begin
        wreg = 1'b0;
        wpcir = 1'b0;  // STALL!
    end
end
```

---

## Major Components

### 1. Register File

32 general-purpose registers with dual read ports and single write port.

```verilog
module RegisterFile(
    input [4:0] rs,           // Read address 1
    input [4:0] rt,           // Read address 2
    input [4:0] wdestReg,     // Write address
    input [31:0] wbData,      // Write data
    input wwreg,              // Write enable
    input clk,
    output reg [31:0] qa,     // Read data 1
    output reg [31:0] qb      // Read data 2
);

    reg [31:0] regfile [0:31];
    
    // Asynchronous read
    always @(*) begin
        qa = regfile[rs];
        qb = regfile[rt];
    end
    
    // Synchronous write
    always @(negedge clk) begin
        if (wwreg && wdestReg != 5'b0)  // Don't write to $0
            regfile[wdestReg] <= wbData;
    end

endmodule
```

**Key Features:**
- Register $0 hardwired to zero
- Write on negative edge (avoids read-write conflicts)
- Dual-port read (can read two registers simultaneously)

---

### 2. Arithmetic Logic Unit (ALU)

Performs arithmetic and logical operations based on control signal.

```verilog
module ALU(
    input [3:0] ealuc,        // ALU control
    input [31:0] eqa,         // Operand A
    input [31:0] b,           // Operand B
    output reg [31:0] r       // Result
);

    always @(*) begin
        case(ealuc)
            4'b0010: r = eqa + b;   // ADD
            4'b0110: r = eqa - b;   // SUB
            4'b0000: r = eqa & b;   // AND
            4'b0001: r = eqa | b;   // OR
            4'b1110: r = eqa ^ b;   // XOR
            default: r = 32'h0;
        endcase
    end

endmodule
```

---

### 3. Pipeline Registers

Registers between stages hold intermediate values and control signals.

**ID/EX Pipeline Register:**
```verilog
module IDEXEReg(
    // Control signals
    input wreg, m2reg, wmem,
    input [3:0] aluc,
    input aluimm,
    input [4:0] destReg,
    
    // Data
    input [31:0] qa, qb, imm32,
    input clk,
    
    // Outputs (latched versions)
    output reg ewreg, em2reg, ewmem,
    output reg [3:0] ealuc,
    output reg ealuimm,
    output reg [4:0] edestReg,
    output reg [31:0] eqa, eqb, eimm32
);

    always @(posedge clk) begin
        ewreg <= wreg;
        em2reg <= m2reg;
        ewmem <= wmem;
        ealuc <= aluc;
        ealuimm <= aluimm;
        edestReg <= destReg;
        eqa <= qa;
        eqb <= qb;
        eimm32 <= imm32;
    end

endmodule
```

---

## Test Program

The CPU executes a test program demonstrating forwarding:

```assembly
# Test Program (loaded into instruction memory)

add $3, $1, $2    # $3 = $1 + $2 = 0xA00000AA + 0x10000011 = 0xB00000BB
sub $4, $9, $3    # $4 = $9 - $3 (requires forwarding from previous instruction)
or  $5, $3, $9    # $5 = $3 | $9 (bitwise OR)
xor $6, $3, $9    # $6 = $3 ^ $9 (bitwise XOR)
and $7, $3, $9    # $7 = $3 & $9 (bitwise AND)
```

**Instruction Encoding:**
```verilog
// add $3, $1, $2
instMemory[25] = {6'b000000, 5'd1, 5'd2, 5'd3, 5'b00000, 6'b100000};

// sub $4, $9, $3
instMemory[26] = {6'b000000, 5'd9, 5'd3, 5'd4, 5'b00000, 6'b100010};
```

---

## FPGA Implementation

### Target Device
**Xilinx XC7Z010-CLG400-1**
- Zynq-7000 SoC
- 28,000 logic cells
- 240 KB block RAM
- 80 DSP slices

### Development Workflow

**1. RTL Design**
- Wrote Verilog modules for each component
- Organized hierarchical design structure

**2. Simulation & Verification**
- Created comprehensive testbench
- Validated all instructions and hazard cases
- Analyzed waveforms for timing correctness

**3. Synthesis**
- Converted Verilog to gate-level netlist
- Xilinx Vivado synthesis tool
- Optimized for area and speed

**4. Implementation**
- Mapped logic to FPGA resources
- Place and route optimization
- Timing closure achieved

**5. Bitstream Generation**
- Created FPGA configuration file
- Ready for hardware programming

**6. Hardware Validation**
- Programmed XC7Z010 FPGA
- Verified operation on physical hardware

---

## Performance Analysis

### Resource Utilization

| Resource | Used | Available | Utilization |
|----------|------|-----------|-------------|
| LUTs | 1,247 | 17,600 | 7% |
| Flip-Flops | 423 | 35,200 | 1.2% |
| Block RAM | 2 | 60 | 3.3% |
| DSP Slices | 0 | 80 | 0% |

**Analysis:** Very low utilization leaves room for future enhancements (cache, branch prediction, etc.)

---

### Timing Performance

**Maximum Clock Frequency:** ~85 MHz (11.8 ns period)

**Critical Path:** Control Unit → Forwarding Mux → ALU → EX/MEM Register

**Throughput:**
- Ideal: 1 instruction per cycle (85 MIPS at 85 MHz)
- With hazards: ~0.8 instructions per cycle (68 MIPS)

**Latency:**
- First instruction: 5 cycles
- Subsequent instructions: 1 cycle each (with forwarding)

---

### Hazard Statistics (Test Program)

| Hazard Type | Occurrences | Resolution Method |
|-------------|-------------|-------------------|
| Data Hazards | 4 | Forwarding (no stall) |
| Load-Use Hazards | 0 | N/A (no loads in test) |
| Structural Hazards | 0 | Separate memories |

**Result:** Zero pipeline stalls in test program due to effective forwarding

---

## Simulation Results

### Waveform Analysis

{% include image-gallery.html images="/assets/images/projects/cpu-waveform.jpg" height="400" %}

**Key Observations:**
- Pipeline fills over first 5 cycles
- Forwarding signals activate when hazards detected
- All instructions complete successfully
- Register values update correctly

### Console Output

```
========================================
5-Stage Pipelined MIPS CPU Simulation
========================================

Time(ns) | PC  | Instruction | EX_Stage | MEM_Stage | WB_Stage
---------|-----|-------------|----------|-----------|----------
     11 | 100 | 00221820 | Reg[ 0]  | Reg[ 0]   | Reg[ 0]
     21 | 104 | 01232022 | Reg[ 3]  | Reg[ 0]   | Reg[ 0]
     31 | 108 | 00692825 | Reg[ 4]  | Reg[ 3]   | Reg[ 0]
     41 | 112 | 00693026 | Reg[ 5]  | Reg[ 4]   | Reg[ 3]
     51 | 116 | 00693824 | Reg[ 6]  | Reg[ 5]   | Reg[ 4]
```

---

## Lessons Learned

### Technical Insights

**Pipeline Design:**
- Separating control and data paths simplifies design
- Pipeline registers are critical for timing closure
- Hazard detection adds significant control complexity

**Verilog Best Practices:**
- Modular design enables easier debugging
- Consistent naming conventions essential
- Extensive commenting crucial for complex designs

**FPGA Development:**
- Simulation catches most bugs before synthesis
- Timing constraints guide implementation
- Resource utilization analysis informs optimization

### Engineering Trade-offs

**Performance vs. Complexity:**
- Forwarding eliminates most stalls but adds mux delays
- More aggressive forwarding could reduce critical path

**Hardware vs. Software:**
- Hardware state machines faster than software equivalents
- HDL debugging more challenging than C debugging

---

## Future Enhancements

**Near-Term Improvements:**
1. Branch prediction (reduce control hazard penalties)
2. Expanded instruction set (shifts, jumps, branches)
3. Exception handling

**Advanced Features:**
4. Instruction cache (reduce memory bottleneck)
5. Data cache (improve memory access speed)
6. Out-of-order execution (superscalar)
7. Multi-cycle instructions (multiplication, division)

**Alternative Implementations:**
- Implement on larger FPGA (Artix-7, Kintex-7)
- Add co-processor for floating-point operations
- Create multi-core version

---

## Project Files

**GitHub Repository:** [github.com/jacobfrancis/fpga-pipelined-mips-cpu](https://github.com/yourusername/fpga-pipelined-mips-cpu)

**Files Included:**
- Complete Verilog source code (all modules)
- Testbench and test vectors
- Vivado project files
- Synthesis and implementation reports
- Waveform screenshots
- Architecture documentation
- Quick start guide

---

## Conclusion

This project successfully demonstrates the fundamental principles of pipelined processor design, from basic instruction fetch through complex hazard resolution. The implementation showcases how modern CPUs achieve high throughput through instruction-level parallelism while managing the inherent challenges of data dependencies.

The combination of forwarding and selective stalling achieves near-ideal pipeline performance, with minimal impact on clock frequency. Deployment to physical FPGA hardware validates the design's correctness and provides a foundation for understanding how software executes at the hardware level.

**Key Takeaway:** Pipelining transforms sequential instruction execution into a parallel assembly line, dramatically improving processor throughput at the cost of increased control complexity.

---

*Project completed as part of Computer Architecture coursework at Morgan State University, Fall 2024. Successfully synthesized and deployed to Xilinx XC7Z010 FPGA.*
