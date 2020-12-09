# NfiVe32
Area Optimized RV32IC CPU core with a single AHB-Lite master Interface for Data and Instructions. 
## features:
- Target clock frequency > 100MHz in 130nm technologies
- Average CPI ~ 4.0 (executing directly from external quad i/o SPI flash)
- ASIC cell count: < 10K (with the tick timer and PIC)
- Instruction Cycles (3/4)
  - C0 : Fetch and Decompress, 
  - C1 : Fetch cyle 2; optional, only used for unaligned 32-bit instructions
  - C2 : RF read, ALU & Branch, 
  - C3 : Memory & RF write-back
- A single AHB-Lite Master interface for both instructions and data
  - Instr: A(C3), I(C0)
  - Data: A(C2), D(C3)

## NfiVe32 Interfacing
<img src=docs/n5_interface.png>

## Directory Structure
- `./dv/` contains the testbench, flash behavioral model and verification automation script.
- `./sw/` contains all needed files to build c and assembly test cases
- `./rtl/` contains the verilog files for:
  - `NfiVe32.v` : CPU core and its components
  - `NfiVe32_SYS.v` : CPU sub-system (CPU, PIC, and SYSTICK timer)
  - `QSPI_XIP_CTRL.v` : QSPI XIP Flash controller with 32x16 direct mapped cache
  - `DFFRFile.v` : Optional area and power optimized Register File (SKY130A)
  - `DMC_32x16HC.v` : Optional area and power optimized 32x16 direct mapped cache (SKY130A)
## Verification
`test.sh` in the `./dv/` folder runs number of c test cases on the CPU core. The script reports the results of each test case as well as the average CPI. To run type : `./test.sh 2> /dev/null`. The testbench for NfiVe32 is a small SoC that contains the CPU, 64KBytes of SRAM *@ 0x20000000) and QSPI flash (@0x00000000) on AHB-Lite bus.