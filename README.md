# N5
Area Optimized RV32IC CPU core with a single AHB-Lite master Interface for Data and Instructions. It has the following features:
	* Target clock frequency > 100MHz in 130nm technologies
	* Average CPI ~ 4.0 (executing directly from external quad i/o SPI flash)
	* ASIC cell count: < 10K (with the tick timer and PIC)
	* Instruction Cycles (3/4)
	    + C0 : Fetch and Decompress, 
	    + C1 : Fetch cyle 2; optional, only used for unaligned 32-bit instructions
	    + C2 : RF read, ALU & Branch, 
	    + C3 : Memory & RF write-back
	* A single AHB-Lite Master interface for both instructions and data
	    + Instr: A(C3), I(C0)
	    + Data: A(C2), D(C3)
