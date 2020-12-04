`timescale 1ns/1ns

`define   TEST_FILE   "../sw/main.hex" 
`define   SIM_TIME    500_000_000
`define   SIM_LEVEL   1

module n5_tb;

reg HCLK, HRESETn;

wire [3:0]		  fdi;
wire [3:0]    	fdo;
wire     	fdoe;
wire          	fsclk;
wire          	fcen;

wire             HSEL_FLASH, HSEL_SRAM, HSEL_DBG;
wire             HREADY;
wire     [1:0]   HTRANS;
wire     [2:0]   HSIZE;
wire             HWRITE;
wire     [31:0]  HADDR;
wire             HREADYOUT, HREADYOUT_FLASH, HREADYOUT_SRAM;
wire     [31:0]  HRDATA, HRDATA_FLASH, HRDATA_SRAM;
wire 	  [31:0]	 HWDATA;

//QSPIXIP qspi
QSPI_XIP_CTRL qspi (
    // Global Signals
    .HCLK(HCLK),
    .HRESETn(HRESETn),

    .din(fdi),
    .dout(fdo),
    .douten(fdoe),
    .sck(fsclk),
    .ce_n(fcen),

    // AHB-Lite Slave Interface
    .HSEL(HSEL_FLASH),
    .HREADY(HREADY),
    .HTRANS(HTRANS),
    //.HSIZE(HSIZE),
    .HWRITE(HWRITE),
    .HADDR(HADDR),
    .HREADYOUT(HREADYOUT_FLASH),
    .HRDATA(HRDATA_FLASH)
);



AHB2MEM SRAM
(
    .HCLK(HCLK),
    .HRESETn(HRESETn),
    .HSEL(HSEL_SRAM),
    .HREADY(HREADY),
    .HTRANS(HTRANS),
    .HSIZE(HSIZE),
    .HWRITE(HWRITE),
    .HADDR(HADDR),
    .HREADYOUT(HREADYOUT_SRAM),
    .HRDATA(HRDATA_SRAM),
    .HWDATA(HWDATA)
    );
//assign HSEL = 1;
//assign HREADY = HREADYOUT;


assign HSEL_FLASH=(HADDR[31:24] == 8'h00);
assign HSEL_SRAM=(HADDR[31:24] == 8'h20);
assign HSEL_DBG=(HADDR[31:24] == 8'h80);

reg [1:0] APHASE_MUX_SEL;	
reg [31:0] APhase_HADDR;
always@ (posedge HCLK or negedge HRESETn)
  begin
    if(!HRESETn)
      APHASE_MUX_SEL <= 2'h0;
    else if(HREADY) begin
    	APhase_HADDR <= HADDR;				
      APHASE_MUX_SEL <= {HSEL_SRAM, HSEL_FLASH};
  	  if(HSEL_FLASH==0 && HSEL_SRAM==0) $display("Access undefined memory (%X)", APhase_HADDR);
  	end
  end

assign HREADYOUT 	= APHASE_MUX_SEL[0] ? HREADYOUT_FLASH 	: 	APHASE_MUX_SEL[1] ? HREADYOUT_SRAM : 1'b1;
assign HRDATA 		= APHASE_MUX_SEL[0] ? HRDATA_FLASH		:	APHASE_MUX_SEL[1] ? HRDATA_SRAM : 32'hDEADBEEF;
assign HREADY 		= HREADYOUT;

wire flag = HREADY & HSEL_DBG & HTRANS[1];//(HRDATA == 32'hDEADBEEF);

wire [3:0] fdio = fdoe ? fdo : 4'bzzzz;
assign fdi = fdio;

sst26wf080b flash(.SCK(fsclk),.SIO(fdio),.CEb(fcen));
/*
CORTEXM0DS CM0(
    // CLOCK AND RESETS ------------------
    .HCLK(HCLK),
    .HRESETn(HRESETn),

    // AHB-LITE MASTER PORT --------------
    .HADDR(HADDR),             // AHB transaction address
    .HBURST(),            // AHB burst: tied to single
    .HMASTLOCK(),         // AHB locked transfer (always zero)
    .HPROT(),             // AHB protection: priv; data or inst
    .HSIZE(HSIZE),        // AHB size: byte, half-word or word
    .HTRANS(HTRANS),      // AHB transfer: non-sequential only
    .HWDATA(HWDATA),            // AHB write-data
    .HWRITE(HWRITE),            // AHB write control
    .HRDATA(HRDATA),      // AHB read-data
    .HREADY(HREADY),      // AHB stall signal
    .HRESP(1'b0),        // AHB error response

    // MISCELLANEOUS ---------------------
    .NMI(1'b0),               // Non-maskable interrupt input
    .IRQ(0),               // Interrupt request inputs
    .TXEV(),              // Event output (SEV executed)
    .RXEV(0),              // Event input
    .LOCKUP(),            // Core is locked-up
    .SYSRESETREQ(),       // System reset request
    .STCLKEN(1'b0),           // SysTick SCLK clock enable
    .STCALIB(26'b0),           // SysTick calibration register value

    // POWER MANAGEMENT ------------------
    .SLEEPING()           // Core and NVIC sleeping
);
*/

NfiVe32 N5_32(
	.HCLK(HCLK),
	.HRESETn(HRESETn),

	// AHB-LITE MASTER PORT for Instructions
	.HADDR(HADDR),             // AHB transaction address
	.HSIZE(HSIZE),             // AHB size: byte, half-word or word
	.HTRANS(HTRANS),            // AHB transfer: non-sequential only
	.HWDATA(HWDATA),            // AHB write-data
	.HWRITE(HWRITE),            // AHB write control
	.HRDATA(HRDATA),            // AHB read-data
	.HREADY(HREADY),            // AHB stall signal
	
	// MISCELLANEOUS 
  	.NMI(1'b0),               // Non-maskable interrupt input
  	.IRQ(1'b0),               // Interrupt request inputs
    .IRQ_NUM(0),
  	.SYSTICKCLK()
);



initial begin
    //Inputs initialization
    HCLK = 0;
    HRESETn = 1'bx;        
    #50;
    HRESETn = 0;
    #100;
    @(posedge HCLK);
    HRESETn <= 1;
end

always #5 HCLK = ~ HCLK;

initial begin
    $dumpfile("n5_tb.vcd");
    $dumpvars(`SIM_LEVEL, n5_tb);
    #`SIM_TIME;
    $finish;
end

always @ (posedge HCLK) 
    if(N5_32.instr_ebreak) begin
      $display("CPI=%d.%0d", N5_32.CSR_CYCLE/N5_32.CSR_INSTRET,(N5_32.CSR_CYCLE%N5_32.CSR_INSTRET)*10/N5_32.CSR_INSTRET );
      $finish;
    end




always @(posedge HCLK)
    if(HTRANS[1] & HREADY & HSEL_FLASH)
      $display("Flash Read A:%X (%0t)", HADDR, $time);
    

initial begin
	#1  $readmemh(`TEST_FILE, flash.I0.memory);
end

endmodule


module AHB2MEM
#(parameter MEMWIDTH = 16)					// SIZE = 64KB = 8 KWords
(
	//AHBLITE INTERFACE
		//Slave Select Signals
			input wire HSEL,
		//Global Signal
			input wire HCLK,
			input wire HRESETn,
		//Address, Control & Write Data
			input wire HREADY,
			input wire [31:0] HADDR,
			input wire [1:0] HTRANS,
			input wire HWRITE,
			input wire [2:0] HSIZE,
			
			input wire [31:0] HWDATA,
		// Transfer Response & Read Data
			output wire HREADYOUT,
			output wire [31:0] HRDATA
);


  assign HREADYOUT = 1'b1; // Always ready

// Registers to store Adress Phase Signals

  reg APhase_HSEL;
  reg APhase_HWRITE;
  reg [1:0] APhase_HTRANS;
  reg [31:0] APhase_HADDR;
  reg [2:0] APhase_HSIZE;

// Memory Array  
  reg [31:0] memory[0:(2**(MEMWIDTH-2)-1)];
  
  initial
  begin
	//$readmemh("../Software/code.hex", memory);
  end

// Sample the Address Phase   
  always @(posedge HCLK or negedge HRESETn)
  begin
	 if(!HRESETn)
	 begin
		APhase_HSEL <= 1'b0;
      APhase_HWRITE <= 1'b0;
      APhase_HTRANS <= 2'b00;
		APhase_HADDR <= 32'h0;
		APhase_HSIZE <= 3'b000;
	 end
    else if(HREADY)
    begin
      APhase_HSEL <= HSEL;
      APhase_HWRITE <= HWRITE;
      APhase_HTRANS <= HTRANS;
		APhase_HADDR <= HADDR;
		APhase_HSIZE <= HSIZE;
    end
  end

// Decode the bytes lanes depending on HSIZE & HADDR[1:0]

  wire tx_byte = ~APhase_HSIZE[1] & ~APhase_HSIZE[0];
  wire tx_half = ~APhase_HSIZE[1] &  APhase_HSIZE[0];
  wire tx_word =  APhase_HSIZE[1];
  
  wire byte_at_00 = tx_byte & ~APhase_HADDR[1] & ~APhase_HADDR[0];
  wire byte_at_01 = tx_byte & ~APhase_HADDR[1] &  APhase_HADDR[0];
  wire byte_at_10 = tx_byte &  APhase_HADDR[1] & ~APhase_HADDR[0];
  wire byte_at_11 = tx_byte &  APhase_HADDR[1] &  APhase_HADDR[0];
  
  wire half_at_00 = tx_half & ~APhase_HADDR[1];
  wire half_at_10 = tx_half &  APhase_HADDR[1];
  
  wire word_at_00 = tx_word;
  
  wire byte0 = word_at_00 | half_at_00 | byte_at_00;
  wire byte1 = word_at_00 | half_at_00 | byte_at_01;
  wire byte2 = word_at_00 | half_at_10 | byte_at_10;
  wire byte3 = word_at_00 | half_at_10 | byte_at_11;

// Writing to the memory

  always @(posedge HCLK)
  begin
	 if(APhase_HSEL & APhase_HWRITE & APhase_HTRANS[1])
	 begin
        $display("Write to SRAM: [%x]=%x (%0t)", APhase_HADDR, HWDATA, $time);
		if(byte0)
			memory[APhase_HADDR[MEMWIDTH:2]][7:0] <= HWDATA[7:0];
		if(byte1)
			memory[APhase_HADDR[MEMWIDTH:2]][15:8] <= HWDATA[15:8];
		if(byte2)
			memory[APhase_HADDR[MEMWIDTH:2]][23:16] <= HWDATA[23:16];
		if(byte3)
			memory[APhase_HADDR[MEMWIDTH:2]][31:24] <= HWDATA[31:24];
	  end
  end
/*
// Reading from memory 
// The first 2 words must be known @ Boot Time   
  wire [31:0] mem_word_0 = 32'h0;       // SP
  wire [31:0] mem_word_1 = 32'h0;       // RESET Vector -- Should point to External Flash
  assign HRDATA =   (APhase_HADDR[MEMWIDTH:2]==30'h0) ? mem_word_0 :
                    (APhase_HADDR[MEMWIDTH:2]==30'h1) ? mem_word_1 :
                    memory[APhase_HADDR[MEMWIDTH:2]];
 */
  assign HRDATA =   memory[APhase_HADDR[MEMWIDTH:2]];

  always @(posedge HCLK)
	  if(APhase_HSEL & ~APhase_HWRITE & APhase_HTRANS[1])
      $display("Read from SRAM: [%x]=%x (%0t)", APhase_HADDR, HRDATA, $time); 
// Diagnostic Signal out
//  assign LED = memory[0][7:0];
  
endmodule
