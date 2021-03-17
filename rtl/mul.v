`define INST_MUL    3'b000
`define INST_MULH   3'b001
`define INST_MULHSU 3'b010
`define INST_MULHU  3'b011

module TCMP(clk, rst, a, s);
    input clk, rst;
    input a;
    output reg s;

    reg z;

    always @(posedge clk  or posedge rst) begin
        if (rst) begin
            //Reset logic goes here.
            s <= 1'b0;
            z <= 1'b0;
        end
        else begin
            //Sequential logic goes here.
            z <= a | z;
            s <= a ^ z;
        end
    end
endmodule

module CSADD(clk, rst, x, y, sum);
    input clk, rst;
    input x, y;
    output reg sum;

    reg sc;

    // Half Adders logic
    wire hsum1, hco1;
    assign hsum1 = y ^ sc;
    assign hco1 = y & sc;

    wire hsum2, hco2;
    assign hsum2 = x ^ hsum1;
    assign hco2 = x & hsum1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            //Reset logic goes here.
            sum <= 1'b0;
            sc <= 1'b0;
        end
        else begin
            //Sequential logic goes here.
            sum <= hsum2;
            sc <= hco1 ^ hco2;
        end
    end
endmodule

module SPM(clk, rst, x, y, p);
    parameter size = 33;
    input clk, rst;
    input y;
    input[size-1:0] x;
    output p;

    wire[size-1:1] pp;
    wire[size-1:0] xy;

    genvar i;

    CSADD csa0 (.clk(clk), .rst(rst), .x(x[0]&y), .y(pp[1]), .sum(p));
    generate for(i=1; i<size-1; i=i+1) begin
        CSADD csa (.clk(clk), .rst(rst), .x(x[i]&y), .y(pp[i+1]), .sum(pp[i]));
    end endgenerate
    TCMP tcmp (.clk(clk), .rst(rst), .a(x[size-1]&y), .s(pp[size-1]));

endmodule
/*
module AHB_SPM #(parameter SIZE=32) (
    input  wire         HCLK,      
    input  wire         HRESETn,   
    input  wire         HSEL,      
    input  wire         HREADY,    
    input  wire [1:0]   HTRANS,    
    input  wire [2:0]   HSIZE,     
    input  wire         HWRITE,    
    input  wire [31:0]  HADDR,     
    input  wire [31:0]  HWDATA,    
    output wire         HREADYOUT, 
    output wire [1:0]   HRESP,
    output wire [31:0]  HRDATA
);

    localparam  X_OFF = 0, Y_OFF = 4, P1_OFF = 8, P2_OFF = 12;
    localparam  S0=0, S1=1, S2=2, S3=3;

    reg [7:0]   AHB_ADDR;    
    wire        ahb_access   = HTRANS[1] & HSEL & HREADY;
    wire        _ahb_write_    = ahb_access &  HWRITE;
    wire        ahb_read     = ahb_access & (~HWRITE);
    reg         AHB_WRITE;
    reg         AHB_READ;

    wire        p;
    reg [31:0]  X, Y, P0, P1; 
    reg [7:0]   CNT, ncnt;

    reg [3:0]   STATE, nstate;

    always @(posedge HCLK or negedge HRESETn)
    if(~HRESETn) begin
        AHB_WRITE   <=  1'b0;
        AHB_READ    <=  1'b0;
        AHB_ADDR    <=  8'b0;
    end
    else begin
        AHB_WRITE   <=  _ahb_write_;
        AHB_READ    <=  ahb_read;
        AHB_ADDR    <=  HADDR[7:0];  
    end

    always @(posedge HCLK or negedge HRESETn)
        if(~HRESETn)
            X  <= 32'b0;
        else if(AHB_WRITE && (AHB_ADDR == X_OFF))
            X <= HWDATA;

    always @(posedge HCLK or negedge HRESETn)
        if(~HRESETn)
            Y  <= 32'b0;
        else if(AHB_WRITE && (AHB_ADDR == Y_OFF))
            Y <= HWDATA;
        else if(STATE==S1) Y <= Y >> 1;

    always @(posedge HCLK or negedge HRESETn)
        if(~HRESETn)
            P0  <= 32'b0;
        else if(STATE==S1)
            P0 <= {p,P0[31:1]};

    always @(posedge HCLK or negedge HRESETn)
        if(~HRESETn)
            STATE  <= S0;
        else 
            STATE <= nstate;

    always @*
        case(STATE)
            S0: if(AHB_WRITE && (AHB_ADDR == Y_OFF)) nstate=S1; else nstate=S0;
            S1: if(CNT==31) nstate=S0; else nstate=S1;
        endcase
    
    always @(posedge HCLK or negedge HRESETn)
        if(~HRESETn)
            CNT  <= 8'd0;
        else 
            CNT <= ncnt;

    always @* begin
        ncnt = 0;
        if(CNT==31) ncnt <= 0;
        else if(STATE==S1) ncnt=CNT+1;
    end

    SPM spm(
      .clk(~HCLK),
      .rst(~HRESETn),
      .x(X),
      .y(Y[0]),
      .p(p)
    );

    assign HREADYOUT = (STATE == S0);

    assign HRDATA = P0;

endmodule
*/
module mul_spm #(parameter size = 32) (
    input clk, 
    input rst_n,
    input [size-1:0] op1, op2,
    input [2:0] op,
    output wire [size-1:0] result,
    output done,
    input start
);
    
    
    reg [size-1:0] Y_reg;
    reg [2*size-1:0] P;
    wire [2*size-1:0] P_n = ~P + 1;
    wire p, y;
    reg [1:0] done_state, done_state_next;
    assign y=Y_reg[0];
    

    reg [size-1:0] mul_op1, mul_op2;
    wire [size-1:0] reg1_data_invert = ~op1 + 1;
    wire [size-1:0] reg2_data_invert = ~op2 + 1;
    always @ (*) begin
        case (op)
                `INST_MUL, `INST_MULHU: begin
                    mul_op1 = op1;
                    mul_op2 = op2;
                end
                `INST_MULHSU: begin
                    mul_op1 = (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 = op2;
                end
                `INST_MULH: begin
                    mul_op1 = (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 = (op2[31] == 1'b1)? (reg2_data_invert): op2;
                end
                default: begin
                    mul_op1 = op1;
                    mul_op2 = op2;
                end
        endcase
    end


    assign done = done_state[1];
    SPM  #(.size(size)) spm(
      .clk(clk),
      .rst(~rst_n),
      .x(mul_op1),
      .y(y),
      .p(p));

    reg [7:0] count;

    always @(posedge clk or negedge rst_n)
      if(!rst_n) count <= 8'd0;
      else if(start) count <= count + 8'd1;
      else if(done) count <= 8'd0;

    always@(posedge clk)
      if(count==0 && start==1) P<= 64'b0;
      else if(!done) P <= {p, P[63:1]};

    always@(posedge clk or negedge rst_n)
      if(!rst_n) Y_reg <= 0;
      else if(count==0 && start==1) Y_reg<= mul_op2;
      else if(!done) Y_reg <= (Y_reg>>1);

    always @(posedge clk or negedge rst_n)
      if(!rst_n) done_state <= 2'd00;
      else done_state <= done_state_next;

    always @* begin
        done_state_next = done_state;
      case (done_state)
        2'b00: if(start) done_state_next = 2'b01;
        2'b01: if(count==8'd65) done_state_next = 2'b10;
        2'b10: done_state_next = 2'b11;
        2'b11: done_state_next = 2'b00;
        default: done_state_next = done_state;
      endcase
    end
      //else if(count==7'd33) done <= 1;
      //else if(start) done <= 0;
   
    assign result = (op == `INST_MUL)    ?   P[31:0]    :
                    (op == `INST_MULHU)  ?   P[63:32]   :
                    (op == `INST_MULH)   ?   ((op1[31] ^ op2[31]) ?  P_n[63:32] : P[63:32] ) :
                    (op == `INST_MULHSU) ?   ((op1[31]) ? P_n[63:32] : P[63:32]) : 32'd0;    
endmodule

/*
module mul #(parameter size = 32) (
    input clk, 
    input rst_n,
    input [size-1:0] op1, op2,
    input [2:0] op,
    output wire [size-1:0] result,
    output done,
    input start
);

    wire[2*size-1:0] mul_temp;
    wire[2*size-1:0] mul_temp_invert;
    
    reg [size-1:0] mul_op1, mul_op2;
    wire [size-1:0] reg1_data_invert = ~op1 + 1;
    wire [size-1:0] reg2_data_invert = ~op2 + 1;
    
    always @ (*) begin
        case (op)
                `INST_MUL, `INST_MULHU: begin
                    mul_op1 = op1;
                    mul_op2 = op2;
                end
                `INST_MULHSU: begin
                    mul_op1 = (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 = op2;
                end
                `INST_MULH: begin
                    mul_op1 = (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 = (op2[31] == 1'b1)? (reg2_data_invert): op2;
                end
                default: begin
                    mul_op1 = op1;
                    mul_op2 = op2;
                end
        endcase
    end

    assign mul_temp = mul_op1 * mul_op2;
    assign mul_temp_invert = ~mul_temp + 1;

    reg [31:0] reg_wdata;
    always @* begin
        case (op)
            `INST_MUL: reg_wdata = mul_temp[31:0];
            `INST_MULHU: reg_wdata = mul_temp[63:32];
            `INST_MULH: 
                case ({op1[31], op2[31]})
                    2'b00: reg_wdata = mul_temp[63:32];
                    2'b11: reg_wdata = mul_temp[63:32];
                    2'b10: reg_wdata = mul_temp_invert[63:32];
                    default: reg_wdata = mul_temp_invert[63:32];
                endcase
            `INST_MULHSU: begin
                if (op1[31] == 1'b1) begin
                    reg_wdata = mul_temp_invert[63:32];
                end else begin
                    reg_wdata = mul_temp[63:32];
                end
            end
        endcase
    end

    assign result = reg_wdata;

    assign done = start;

endmodule
*/

module mul #(parameter size = 32) (
    input clk, 
    input rst_n,
    input [size-1:0] op1, op2,
    input [2:0] op,
    output wire [size-1:0] result,
    output done,
    input start
);

    reg[2*size-1:0] mul_temp;
    reg[2*size-1:0] mul_temp_invert;
    
    reg [size-1:0] mul_op1, mul_op2;
    wire [size-1:0] reg1_data_invert = ~op1 + 1;
    wire [size-1:0] reg2_data_invert = ~op2 + 1;

    reg [1:0] cntr;
    always @(posedge clk or negedge rst_n)
        if(~rst_n) cntr <= 2'b00;
        else 
            if(start) 
                cntr <= cntr + 1'b01; 
            else 
                cntr <= 2'b00;

    always @ (posedge clk) if(start) begin
        case (op)
                `INST_MUL, `INST_MULHU: begin
                    mul_op1 <= op1;
                    mul_op2 <= op2;
                end
                `INST_MULHSU: begin
                    mul_op1 <= (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 <= op2;
                end
                `INST_MULH: begin
                    mul_op1 <= (op1[31] == 1'b1)? (reg1_data_invert): op1;
                    mul_op2 <= (op2[31] == 1'b1)? (reg2_data_invert): op2;
                end
                default: begin
                    mul_op1 <= op1;
                    mul_op2 <= op2;
                end
        endcase
    end

    
    always @(posedge clk) begin
        mul_temp <= mul_op1 * mul_op2;
        mul_temp_invert <= ~mul_temp + 1;
    end
    //assign mul_temp = mul_op1 * mul_op2;
    //assign mul_temp_invert = ~mul_temp + 1;

    reg [31:0] reg_wdata;
    always @* begin
        case (op)
            `INST_MUL: reg_wdata = mul_temp[31:0];
            `INST_MULHU: reg_wdata = mul_temp[63:32];
            `INST_MULH: 
                case ({op1[31], op2[31]})
                    2'b00: reg_wdata = mul_temp[63:32];
                    2'b11: reg_wdata = mul_temp[63:32];
                    2'b10: reg_wdata = mul_temp_invert[63:32];
                    default: reg_wdata = mul_temp_invert[63:32];
                endcase
            `INST_MULHSU: begin
                if (op1[31] == 1'b1) begin
                    reg_wdata = mul_temp_invert[63:32];
                end else begin
                    reg_wdata = mul_temp[63:32];
                end
            end
        endcase
    end
    
    assign result = reg_wdata;

    //assign done = start;
    assign done = (cntr == 2'b10);
endmodule

/*
module spm_tb;
reg clk, rst, start;
wire p;


wire done;

initial begin
  rst = 0;
  clk = 0;

  start = 0;
  #100;
  rst = 1;
  #500;
  rst = 0;
  #1000;
  @(posedge clk);
  start = 1;
  @(posedge done);
  start = 0;
end

always #10 clk = ~clk;

initial begin
  $dumpfile("spm.vcd");
  $dumpvars(0);
  #100_000 ;
  $display("Timeout -- Exiting");
  $finish;
end

wire [31:0] P;

SPM_EXT spm_dut(.clk(clk), .rst(rst), .X(-15), .Y(20), .P(P), .start(start), .done(done));

endmodule
*/


/*

module picorv32_pcpi_mul #(
	parameter STEPS_AT_ONCE = 1,
	parameter CARRY_CHAIN = 4
) (
	input clk, resetn,

	input             pcpi_valid,
	input      [31:0] pcpi_insn,
	input      [31:0] pcpi_rs1,
	input      [31:0] pcpi_rs2,
	output reg        pcpi_wr,
	output reg [31:0] pcpi_rd,
	output reg        pcpi_wait,
	output reg        pcpi_ready
);
	reg instr_mul, instr_mulh, instr_mulhsu, instr_mulhu;
	wire instr_any_mul = |{instr_mul, instr_mulh, instr_mulhsu, instr_mulhu};
	wire instr_any_mulh = |{instr_mulh, instr_mulhsu, instr_mulhu};
	wire instr_rs1_signed = |{instr_mulh, instr_mulhsu};
	wire instr_rs2_signed = |{instr_mulh};

	reg pcpi_wait_q;
	wire mul_start = pcpi_wait && !pcpi_wait_q;

	always @(posedge clk) begin
		instr_mul <= 0;
		instr_mulh <= 0;
		instr_mulhsu <= 0;
		instr_mulhu <= 0;

		if (resetn && pcpi_valid && pcpi_insn[6:0] == 7'b0110011 && pcpi_insn[31:25] == 7'b0000001) begin
			case (pcpi_insn[14:12])
				3'b000: instr_mul <= 1;
				3'b001: instr_mulh <= 1;
				3'b010: instr_mulhsu <= 1;
				3'b011: instr_mulhu <= 1;
			endcase
		end

		pcpi_wait <= instr_any_mul;
		pcpi_wait_q <= pcpi_wait;
	end

	reg [63:0] rs1, rs2, rd, rdx;
	reg [63:0] next_rs1, next_rs2, this_rs2;
	reg [63:0] next_rd, next_rdx, next_rdt;
	reg [6:0] mul_counter;
	reg mul_waiting;
	reg mul_finish;
	integer i, j;

	// carry save accumulator
	always @* begin
		next_rd = rd;
		next_rdx = rdx;
		next_rs1 = rs1;
		next_rs2 = rs2;

		for (i = 0; i < STEPS_AT_ONCE; i=i+1) begin
			this_rs2 = next_rs1[0] ? next_rs2 : 0;
			if (CARRY_CHAIN == 0) begin
				next_rdt = next_rd ^ next_rdx ^ this_rs2;
				next_rdx = ((next_rd & next_rdx) | (next_rd & this_rs2) | (next_rdx & this_rs2)) << 1;
				next_rd = next_rdt;
			end else begin
				next_rdt = 0;
				for (j = 0; j < 64; j = j + CARRY_CHAIN)
					{next_rdt[j+CARRY_CHAIN-1], next_rd[j +: CARRY_CHAIN]} =
							next_rd[j +: CARRY_CHAIN] + next_rdx[j +: CARRY_CHAIN] + this_rs2[j +: CARRY_CHAIN];
				next_rdx = next_rdt << 1;
			end
			next_rs1 = next_rs1 >> 1;
			next_rs2 = next_rs2 << 1;
		end
	end

	always @(posedge clk) begin
		mul_finish <= 0;
		if (!resetn) begin
			mul_waiting <= 1;
		end else
		if (mul_waiting) begin
			if (instr_rs1_signed)
				rs1 <= $signed(pcpi_rs1);
			else
				rs1 <= $unsigned(pcpi_rs1);

			if (instr_rs2_signed)
				rs2 <= $signed(pcpi_rs2);
			else
				rs2 <= $unsigned(pcpi_rs2);

			rd <= 0;
			rdx <= 0;
			mul_counter <= (instr_any_mulh ? 63 - STEPS_AT_ONCE : 31 - STEPS_AT_ONCE);
			mul_waiting <= !mul_start;
		end else begin
			rd <= next_rd;
			rdx <= next_rdx;
			rs1 <= next_rs1;
			rs2 <= next_rs2;

			mul_counter <= mul_counter - STEPS_AT_ONCE;
			if (mul_counter[6]) begin
				mul_finish <= 1;
				mul_waiting <= 1;
			end
		end
	end

	always @(posedge clk) begin
		pcpi_wr <= 0;
		pcpi_ready <= 0;
		if (mul_finish && resetn) begin
			pcpi_wr <= 1;
			pcpi_ready <= 1;
			pcpi_rd <= instr_any_mulh ? rd >> 32 : rd;
		end
	end
endmodule
*/