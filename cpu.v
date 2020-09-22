//cpu
module thirdcpu(clk, rst);
input wire clk, rst;
wire [31:0]newaddr;
wire [31:0]addr;
wire [31:0]finaladdr;
wire [31:0] instr;
wire [15:0] sign;
wire [4:0]rd;
wire [4:0]rt;
wire [4:0]rs;
wire [5:0]op;
wire [63:0]out1;
wire [63:0]out2;
wire [63:0]signout;
wire Regenable, ALUenable, Dataenable, datarw;
wire [63:0]dataOut;
wire eq, MUX3, MUX2;
wire [63:0]ALUout;
wire [31:0] offset; 
wire [31:0]addr_plus_sign;
wire jumpflag;
assign addr_plus_sign = finaladdr + 1 + offset;
assign newaddr = finaladdr + 1;

PC pc(clk, rst, op, offset, addr, signout, finaladdr);
instrmemory im(clk, rst, finaladdr, sign, rd, rt, rs, instr,op);
registerfile rf(clk, rst, rd, rt, rs, dataOut, ALUout, MUX2, Regenable, out1, out2);
ALU alu(ALUenable, rs, MUX3, out1, signout, out2, eq, ALUout);
controlunit control(op, eq, Regenable, ALUenable, Dataenable, datarw, MUX2, MUX3, jumpflag);
signextender signextend(sign, signout, offset);
datamem dataM(clk, rst, ALUout, out2, Dataenable, datarw, dataOut);
mux32bit pcmux(newaddr, addr_plus_sign, jumpflag, addr);

endmodule

//pc
module PC(clk, rst, op, offset, addr, signout, finaladdr);
input wire [31:0]offset;
input wire clk, rst; 
reg [31:0]addr_plus_sign;
input [5:0] op;
input [63:0]signout;                       
output reg [31:0]finaladdr; 
input wire [31:0]addr;

initial begin
  finaladdr <= 0;  
end

always@(posedge clk) begin
        if (rst == 1)  begin
        finaladdr <= 0;  
        end
    else begin
        finaladdr <= addr;
        $display("next addr", addr);
      end
    end
endmodule

//instrmem
module instrmemory(clk, rst, getaddr, sign, rd, rt, rs, instr,op);
input wire clk, rst;
input wire [31:0] getaddr;
output wire[15:0] sign;
output wire[4:0]rd;
output wire[4:0]rt;
output wire[4:0]rs;
output wire[5:0]op;
output reg [31:0]instr;

reg [31:0] mem[0:65535];

reg [31:0] address;

assign sign = instr[15:0];
assign rd = instr[25:21];
assign rt = instr[20:16];
assign rs = instr[25:21];
assign op = instr[31:26];

always @(posedge clk) begin
    if(rst ==1) begin
        $readmemh ("cmp.ja.mc", mem); 
    end
end
always @(posedge clk) begin
	instr = mem[getaddr];
  $display("op = %b, sign = %b, rd = %b, rt = %b, rs = %b", op, sign, rd, rt, rs);
end
endmodule


//datamem
module datamem(clk, rst, Ina, Inb, enable, readwrite, dataOut);
input wire clk;
input wire rst;
input wire [63:0] Ina;
input wire [63:0] Inb;
input wire enable;
input wire readwrite;
reg [31:0] memory[0:65535];
output reg [63:0] dataOut;

always @(posedge clk) begin
    if(rst ==1) 
      $readmemh ("cmp.ja.mc", memory); 
end
always@(posedge clk or enable or Ina) begin
    if (enable == 1 && readwrite ==0) begin
    dataOut <= memory[Ina];
  end
  else if(enable == 1 && readwrite == 1) begin
      memory[Ina] <= Inb; 
  end
end
endmodule

//register
module registerfile (clk, rst, rd, rt, rs, dataOut, ALUout, MUX2, writeenable, out1, out2);
input writeenable, clk, rst, MUX2;
input [4:0] rd;
input [4:0] rt;
input [4:0] rs;
input [63:0] dataOut;
input [63:0] ALUout;
output [63:0] out1, out2;
reg [31:0] RF [63:0];
wire [4:0] writeReg;
wire [63:0] writeData;

assign writeReg = writeenable? rd : rt;
assign writeData = MUX2? ALUout : dataOut;

always @(posedge clk) begin
      if (rst ==1) begin
      RF[0] <= 0;
	    RF[1] <= 0;
	    RF[2] <= 0;
	    RF[3] <= 0;
	    RF[4] <= 0;
	    RF[5] <= 0;
	    RF[6] <= 0;
	    RF[7] <= 0;
      RF[8] <= 0;
	    RF[9] <= 0;
	    RF[10] <= 0;
	    RF[11] <= 0;
	    RF[12] <= 0;
	    RF[13] <= 0;
	    RF[14] <= 0;
      RF[15] <= 0;
	    RF[16] <= 0;
	    RF[17] <= 0;
	    RF[18] <= 0;
	    RF[19] <= 0;
	    RF[20] <= 0;
	    RF[21] <= 0;
      RF[22] <= 0;
	    RF[23] <= 0;
	    RF[24] <= 0;
	    RF[25] <= 0;
	    RF[26] <= 0;
	    RF[27] <= 0;
	    RF[28] <= 0;
      RF[29] <= 0;
	    RF[30] <= 0;
	    RF[31] <= 0;
	    RF[32] <= 0;
	    RF[33] <= 0;
	    RF[34] <= 0;
	    RF[35] <= 0;
      RF[36] <= 0;
	    RF[37] <= 0;
	    RF[38] <= 0;
      RF[39] <= 0;
	    RF[40] <= 0;
      RF[41] <= 0;
      RF[42] <= 0;
	    RF[43] <= 0;
	    RF[44] <= 0;
	    RF[45] <= 0;
	    RF[46] <= 0;
	    RF[47] <= 0;
	    RF[48] <= 0;
      RF[49] <= 0;
	    RF[50] <= 0;
	    RF[51] <= 0;
	    RF[52] <= 0;
	    RF[53] <= 0;
	    RF[54] <= 0;
	    RF[55] <= 0;
      RF[56] <= 0;
	    RF[57] <= 0;
	    RF[58] <= 0;
      RF[59] <= 0;
	    RF[60] <= 0;
      RF[61] <= 0;
	    RF[62] <= 0;
	    RF[63] <= 0;
      end 
  end
always@(posedge clk)
    begin
    if (writeenable && writeReg) begin
		  RF[writeReg] <= writeData;
      // DPW, this should display the loaded number.
      $display("writing into RF:%d\n", writeReg);
    end
end
always@(RF[1] or RF[2] or RF[3]) begin
    $display("RF[1] = %d\n", RF[1]);
    $display("RF[2] = %d\n", RF[2]);
    $display("RF[3] = %d\n", RF[3]);
end
assign out1 = RF[rd];
assign out2 = RF[rt];
endmodule

//sign extender
module signextender(in, out64, out32);
input wire [15:0]in;
output wire [63:0]out64;
output wire [31:0]out32;
assign out64 = {{48{in[15]}},in[15:0]};
assign out32 = {{16{in[15]}},in[15:0]};
endmodule

//control unit
module controlunit(opCode, eq, Regenable, ALU, Dataenable, datarw, MUX2, MUX3, jumpflag);
input wire [5:0]opCode;
input eq;
output wire Regenable, ALU, Dataenable, datarw, MUX2, MUX3, jumpflag;
wire [63:0]Y;

decoder6x64 decode(opCode,Y);
rom rom(Y, eq, Regenable, ALU, Dataenable, datarw, MUX2, MUX3, jumpflag);
endmodule

//rom
module rom(in, eq, Regenable, ALU, Dataenable, datarw, MUX2, MUX3, jumpflag);
input wire [63:0]in;
input wire eq;
output reg Regenable, ALU, Dataenable, datarw, MUX2, MUX3; 
output reg jumpflag;
initial begin
jumpflag = 0;
end
//8 instructions
//add: 000100
//and: 100100
//cmp: 111100
//lw: 001011
//sw: 001100
//ja: 001111
//noop: 000000
//halt: 111111
always@(in or eq) begin
jumpflag = (in == 2382987021654430226 && eq || in ==64'b0001001000010010000100100010000100100001000100100001001000010010 && eq)? 1 : 0;
//add, and
MUX2 = (in == 1302123119675445778 || in == 1302686061038932498) ? 1 : 0;
//add, and, cmp
MUX3 = (in == 1302123119675445778 || in == 1302686061038932498 || in == 2382987021654430226) ? 1 : 0;
//add, and, lw
Regenable = (in == 1302123119675445778 || in == 1302686061038932498 || in == 1302123175509893394) ? 1 : 0;
//and
ALU = (in == 1302686061038932498) ? 1 : 0;
//lw, sw
Dataenable = (in == 1302123175509893394 || in == 1302123175510020626) ? 1 : 0;
//sw
datarw = (in == 1302123175510020626) ? 1 : 0;
end 
endmodule


//decoder
module decoder2x4 (A, B, Y0, Y1, Y2, Y3);
input wire A,B;
output wire Y0, Y1, Y2, Y3;

wire E1, E2, E3, E4;

not #1 g1(E1, A);
not #1 g2(E2, B);
not #1 g3(E3, B);
not #1 g4(E4, A);

and #2 g5(Y0, E1, E2);
and #2 g6(Y1, A, E3);
and #2 g7(Y2, E4, B);
and #2 g8(Y3, A, B);

endmodule


module decoder3x8 (A, B, C, Y[7:0]);
input wire A, B, C;
output wire [7:0]Y;
wire E1, E2, E3;

not #1 g1(E1, A);
not #1 g2(E2, B);
not #1 g3(E3, C);
and #2 g4(Y[0], E1, E2, E3);
and #2 g5(Y[1], A, E2, E3);
and #2 g6(Y[2], E1, B, E3);
and #2 g7(Y[3], A, B, E3);
and #2 g8(Y[4], E1, E2, C);
and #2 g9(Y[5], A, E2, C);
and #2 g10(Y[6], E1, B, C);
and #2 g11(Y[7], A, B, C);
endmodule


//6x64 using 2x4 and 3x8 
module decoder6x64 (opCode, Y);
input wire [5:0]opCode;
output wire [63:0]Y;
wire [15:0]E;
wire [31:0]X;

decoder3x8 #3 g1(opCode[0],opCode[1],opCode[2], E[7:0]);
decoder3x8 #3 g2(opCode[3],opCode[4],opCode[5], E[15:8]);

decoder2x4 #3 g3(E[0], E[1], X[0], X[1], X[2], X[3]);
decoder2x4 #3 g4(E[2], E[3], X[4], X[5], X[6], X[7]);
decoder2x4 #3 g5(E[4], E[5], X[8], X[9], X[10], X[11]);
decoder2x4 #3 g6(E[6], E[7], X[12], X[13], X[14], X[15]);
decoder2x4 #3 g7(E[8], E[9], X[16], X[17], X[18], X[19]);
decoder2x4 #3 g8(E[10], E[11], X[20], X[21], X[22], X[23]);
decoder2x4 #3 g9(E[12], E[13], X[24], X[25], X[26], X[27]);
decoder2x4 #3 g10(E[14], E[15], X[28], X[29], X[30], X[31]);

decoder2x4 #3 g11(X[0], X[1], Y[0], Y[1], Y[2], Y[3]);
decoder2x4 #3 g12(X[2], X[3], Y[4], Y[5], Y[6], Y[7]);
decoder2x4 #3 g13(X[4], X[5], Y[8], Y[9], Y[10], Y[11]);
decoder2x4 #3 g14(X[6], X[7], Y[12], Y[13], Y[14], Y[15]);
decoder2x4 #3 g15(X[8], X[9], Y[16], Y[17], Y[18], Y[19]);
decoder2x4 #3 g16(X[10], X[11], Y[20], Y[21], Y[22], Y[23]);
decoder2x4 #3 g17(X[12], X[13], Y[24], Y[25], Y[26], Y[27]);
decoder2x4 #3 g18(X[14], X[15], Y[28], Y[29], Y[30], Y[31]);
decoder2x4 #3 g19(X[16], X[17], Y[32], Y[33], Y[34], Y[35]);
decoder2x4 #3 g20(X[18], X[19], Y[36], Y[37], Y[38], Y[39]);
decoder2x4 #3 g21(X[20], X[21], Y[40], Y[41], Y[42], Y[43]);
decoder2x4 #3 g22(X[22], X[23], Y[44], Y[45], Y[46], Y[47]);
decoder2x4 #3 g23(X[24], X[25], Y[48], Y[49], Y[50], Y[51]);
decoder2x4 #3 g24(X[26], X[27], Y[52], Y[53], Y[54], Y[55]);
decoder2x4 #3 g25(X[28], X[29], Y[56], Y[57], Y[58], Y[59]);
decoder2x4 #3 g26(X[30], X[31], Y[60], Y[61], Y[62], Y[63]);
endmodule

//alu
module ALU(op, rs, MUX3, out1, signout, out2, eq, ALUout);
    input wire op, MUX3;
    input wire[4:0]rs;         
    input wire [63:0] out1; 
    input wire [63:0] out2;             
    input wire [63:0] signout;              
    output wire eq;       
    output [63:0] ALUout;   
    wire [63:0]B;
    assign B = MUX3? out2 : signout;

    /*mux64bit mux1(Sout, Aout, op, ALUout);*/
    comparing64bit equal(out1, out2, eq);
    assign ALUout = op? out1&B: out1+B;
endmodule

//mux
module mux64bit(A,B,S,Y);
input wire [63:0]A;
input wire [63:0]B;
input wire S;
output wire [63:0]Y;
wire E1, E2, E3;

mux #5 g_1(A[0], B[0], S, Y[0]);
mux #5 g_2(A[1], B[1], S, Y[1]);
mux #5 g_3(A[2], B[2], S, Y[2]);
mux #5 g_4(A[3], B[3], S, Y[3]);
mux #5 g_5(A[4], B[4], S, Y[4]);
mux #5 g_6(A[5], B[5], S, Y[5]);
mux #5 g_7(A[6], B[6], S, Y[6]);
mux #5 g_8(A[7], B[7], S, Y[7]);
mux #5 g_9(A[8], B[8], S, Y[8]);
mux #5 g_10(A[9], B[9], S, Y[9]);
mux #5 g_11(A[10], B[10], S, Y[10]);
mux #5 g_12(A[11], B[11], S, Y[11]);
mux #5 g_13(A[12], B[12], S, Y[12]);
mux #5 g_14(A[13], B[13], S, Y[13]);
mux #5 g_15(A[14], B[14], S, Y[14]);
mux #5 g_16(A[15], B[15], S, Y[15]);
mux #5 g_17(A[16], B[16], S, Y[16]);
mux #5 g_18(A[17], B[17], S, Y[17]);
mux #5 g_19(A[18], B[18], S, Y[18]);
mux #5 g_20(A[19], B[19], S, Y[19]);
mux #5 g_21(A[20], B[20], S, Y[20]);
mux #5 g_22(A[21], B[21], S, Y[21]);
mux #5 g_23(A[22], B[22], S, Y[22]);
mux #5 g_24(A[23], B[23], S, Y[23]);
mux #5 g_25(A[24], B[24], S, Y[24]);
mux #5 g_26(A[25], B[25], S, Y[25]);
mux #5 g_27(A[26], B[26], S, Y[26]);
mux #5 g_28(A[27], B[27], S, Y[27]);
mux #5 g_29(A[28], B[28], S, Y[28]);
mux #5 g_30(A[29], B[29], S, Y[29]);
mux #5 g_31(A[30], B[30], S, Y[30]);
mux #5 g_32(A[31], B[31], S, Y[31]);
mux #5 g_33(A[32], B[32], S, Y[32]);
mux #5 g_34(A[33], B[33], S, Y[33]);
mux #5 g_35(A[34], B[34], S, Y[34]);
mux #5 g_36(A[35], B[35], S, Y[35]);
mux #5 g_37(A[36], B[36], S, Y[36]);
mux #5 g_38(A[37], B[37], S, Y[37]);
mux #5 g_39(A[38], B[38], S, Y[38]);
mux #5 g_40(A[39], B[39], S, Y[39]);
mux #5 g_41(A[40], B[40], S, Y[40]);
mux #5 g_42(A[41], B[41], S, Y[41]);
mux #5 g_43(A[42], B[42], S, Y[42]);
mux #5 g_44(A[43], B[43], S, Y[43]);
mux #5 g_45(A[44], B[44], S, Y[44]);
mux #5 g_46(A[45], B[45], S, Y[45]);
mux #5 g_47(A[46], B[46], S, Y[46]);
mux #5 g_48(A[47], B[47], S, Y[47]);
mux #5 g_49(A[48], B[48], S, Y[48]);
mux #5 g_50(A[49], B[49], S, Y[49]);
mux #5 g_51(A[50], B[50], S, Y[50]);
mux #5 g_52(A[51], B[51], S, Y[51]);
mux #5 g_53(A[52], B[52], S, Y[52]);
mux #5 g_54(A[53], B[53], S, Y[53]);
mux #5 g_55(A[54], B[54], S, Y[54]);
mux #5 g_56(A[55], B[55], S, Y[55]);
mux #5 g_57(A[56], B[56], S, Y[56]);
mux #5 g_58(A[57], B[57], S, Y[57]);
mux #5 g_59(A[58], B[58], S, Y[58]);
mux #5 g_60(A[59], B[59], S, Y[59]);
mux #5 g_61(A[60], B[60], S, Y[60]);
mux #5 g_62(A[61], B[61], S, Y[61]);
mux #5 g_63(A[62], B[62], S, Y[62]);
mux #5 g_64(A[63], B[63], S, Y[63]);

endmodule

module mux32bit(A,B,S,Y);
input wire [31:0]A;
input wire [31:0]B;
input wire S;
output wire [31:0]Y;

mux #5 g_1(A[0], B[0], S, Y[0]);
mux #5 g_2(A[1], B[1], S, Y[1]);
mux #5 g_3(A[2], B[2], S, Y[2]);
mux #5 g_4(A[3], B[3], S, Y[3]);
mux #5 g_5(A[4], B[4], S, Y[4]);
mux #5 g_6(A[5], B[5], S, Y[5]);
mux #5 g_7(A[6], B[6], S, Y[6]);
mux #5 g_8(A[7], B[7], S, Y[7]);
mux #5 g_9(A[8], B[8], S, Y[8]);
mux #5 g_10(A[9], B[9], S, Y[9]);
mux #5 g_11(A[10], B[10], S, Y[10]);
mux #5 g_12(A[11], B[11], S, Y[11]);
mux #5 g_13(A[12], B[12], S, Y[12]);
mux #5 g_14(A[13], B[13], S, Y[13]);
mux #5 g_15(A[14], B[14], S, Y[14]);
mux #5 g_16(A[15], B[15], S, Y[15]);
mux #5 g_17(A[16], B[16], S, Y[16]);
mux #5 g_18(A[17], B[17], S, Y[17]);
mux #5 g_19(A[18], B[18], S, Y[18]);
mux #5 g_20(A[19], B[19], S, Y[19]);
mux #5 g_21(A[20], B[20], S, Y[20]);
mux #5 g_22(A[21], B[21], S, Y[21]);
mux #5 g_23(A[22], B[22], S, Y[22]);
mux #5 g_24(A[23], B[23], S, Y[23]);
mux #5 g_25(A[24], B[24], S, Y[24]);
mux #5 g_26(A[25], B[25], S, Y[25]);
mux #5 g_27(A[26], B[26], S, Y[26]);
mux #5 g_28(A[27], B[27], S, Y[27]);
mux #5 g_29(A[28], B[28], S, Y[28]);
mux #5 g_30(A[29], B[29], S, Y[29]);
mux #5 g_31(A[30], B[30], S, Y[30]);
mux #5 g_32(A[31], B[31], S, Y[31]);
endmodule

module mux (A, B, S, Y);
  input wire A,B,S;
  output wire Y;

  wire E1, E2, E3;

  not #1 g1(E1, S);
  and #2 g2(E2,E1,A);
  and #2 g3(E3,S,B);
  or  #2 g4(Y,E2,E3);
endmodule


//eq
module comparing64bit (A, B, eq);
  input wire [63:0]A;
  input wire[63:0]B;
  output wire eq;

  wire [63:0]E;
  wire[63:0]Y;

  xor #3 g1(E[0], A[0], B[0]);
  xor #3 g2(E[1], A[1], B[1]);
  xor #3 g3(E[2], A[2], B[2]);
  xor #3 g4(E[3], A[3], B[3]);
  xor #3 g5(E[4], A[4], B[4]);
  xor #3 g6(E[5], A[5], B[5]);
  xor #3 g7(E[6], A[6], B[6]);
  xor #3 g8(E[7], A[7], B[7]);
  xor #3 g9(E[8], A[8], B[8]);
  xor #3 g10(E[9], A[9], B[9]);
  xor #3 g11(E[10], A[10], B[10]);
  xor #3 g12(E[11], A[11], B[11]);
  xor #3 g13(E[12], A[12], B[12]);
  xor #3 g14(E[13], A[13], B[13]);
  xor #3 g15(E[14], A[14], B[14]);
  xor #3 g16(E[15], A[15], B[15]);
  xor #3 g17(E[16], A[16], B[16]);
  xor #3 g18(E[17], A[17], B[17]);
  xor #3 g19(E[18], A[18], B[18]);
  xor #3 g20(E[19], A[19], B[19]);
  xor #3 g21(E[20], A[20], B[20]);
  xor #3 g22(E[21], A[21], B[21]);
  xor #3 g23(E[22], A[22], B[22]);
  xor #3 g24(E[23], A[23], B[23]);
  xor #3 g25(E[24], A[24], B[24]);
  xor #3 g26(E[25], A[25], B[25]);
  xor #3 g27(E[26], A[26], B[26]);
  xor #3 g28(E[27], A[27], B[27]);
  xor #3 g29(E[28], A[28], B[28]);
  xor #3 g30(E[29], A[29], B[29]);
  xor #3 g31(E[30], A[30], B[30]);
  xor #3 g32(E[31], A[31], B[31]);
  xor #3 g33(E[32], A[32], B[32]);
  xor #3 g34(E[33], A[33], B[33]);
  xor #3 g35(E[34], A[34], B[34]);
  xor #3 g36(E[35], A[35], B[35]);
  xor #3 g37(E[36], A[36], B[36]);
  xor #3 g38(E[37], A[37], B[37]);
  xor #3 g39(E[38], A[38], B[38]);
  xor #3 g40(E[39], A[39], B[39]);
  xor #3 g41(E[40], A[40], B[40]);
  xor #3 g42(E[41], A[41], B[41]);
  xor #3 g43(E[42], A[42], B[42]);
  xor #3 g44(E[43], A[43], B[43]);
  xor #3 g45(E[44], A[44], B[44]);
  xor #3 g46(E[45], A[45], B[45]);
  xor #3 g47(E[46], A[46], B[46]);
  xor #3 g48(E[47], A[47], B[47]);
  xor #3 g49(E[48], A[48], B[48]);
  xor #3 g50(E[49], A[49], B[49]);
  xor #3 g51(E[50], A[50], B[50]);
  xor #3 g52(E[51], A[51], B[51]);
  xor #3 g53(E[52], A[52], B[52]);
  xor #3 g54(E[53], A[53], B[53]);
  xor #3 g55(E[54], A[54], B[54]);
  xor #3 g56(E[55], A[55], B[55]);
  xor #3 g57(E[56], A[56], B[56]);
  xor #3 g58(E[57], A[57], B[57]);
  xor #3 g59(E[58], A[58], B[58]);
  xor #3 g60(E[59], A[59], B[59]);
  xor #3 g61(E[60], A[60], B[60]);
  xor #3 g62(E[61], A[61], B[61]);
  xor #3 g63(E[62], A[62], B[62]);
  xor #3 g64(E[63], A[63], B[63]);

  not  #1 g_1(Y[0], E[0]);
  not  #1 g_2(Y[1], E[1]);
  not  #1 g_3(Y[2], E[2]);
  not  #1 g_4(Y[3], E[3]);
  not  #1 g_5(Y[4], E[4]);
  not  #1 g_6(Y[5], E[5]);
  not  #1 g_7(Y[6], E[6]);
  not  #1 g_8(Y[7], E[7]);
  not  #1 g_9(Y[8], E[8]);
  not  #1 g_10(Y[9], E[9]);
  not  #1 g_11(Y[10], E[10]);
  not  #1 g_12(Y[11], E[11]);
  not  #1 g_13(Y[12], E[12]);
  not  #1 g_14(Y[13], E[13]);
  not  #1 g_15(Y[14], E[14]);
  not  #1 g_16(Y[15], E[15]);
  not  #1 g_17(Y[16], E[16]);
  not  #1 g_18(Y[17], E[17]);
  not  #1 g_19(Y[18], E[18]);
  not  #1 g_20(Y[19], E[19]);
  not  #1 g_21(Y[20], E[20]);
  not  #1 g_22(Y[21], E[21]);
  not  #1 g_23(Y[22], E[22]);
  not  #1 g_24(Y[23], E[23]);
  not  #1 g_25(Y[24], E[24]);
  not  #1 g_26(Y[25], E[25]);
  not  #1 g_27(Y[26], E[26]);
  not  #1 g_28(Y[27], E[27]);
  not  #1 g_29(Y[28], E[28]);
  not  #1 g_30(Y[29], E[29]);
  not  #1 g_31(Y[30], E[30]);
  not  #1 g_32(Y[31], E[31]);
  not  #1 g_33(Y[32], E[32]);
  not  #1 g_34(Y[33], E[33]);
  not  #1 g_35(Y[34], E[34]);
  not  #1 g_36(Y[35], E[35]);
  not  #1 g_37(Y[36], E[36]);
  not  #1 g_38(Y[37], E[37]);
  not  #1 g_39(Y[38], E[38]);
  not  #1 g_40(Y[39], E[39]);
  not  #1 g_41(Y[40], E[40]);
  not  #1 g_42(Y[41], E[41]);
  not  #1 g_43(Y[42], E[42]);
  not  #1 g_44(Y[43], E[43]);
  not  #1 g_45(Y[44], E[44]);
  not  #1 g_46(Y[45], E[45]);
  not  #1 g_47(Y[46], E[46]);
  not  #1 g_48(Y[47], E[47]);
  not  #1 g_49(Y[48], E[48]);
  not  #1 g_50(Y[49], E[49]);
  not  #1 g_51(Y[50], E[50]);
  not  #1 g_52(Y[51], E[51]);
  not  #1 g_53(Y[52], E[52]);
  not  #1 g_54(Y[53], E[53]);
  not  #1 g_55(Y[54], E[54]);
  not  #1 g_56(Y[55], E[55]);
  not  #1 g_57(Y[56], E[56]);
  not  #1 g_58(Y[57], E[57]);
  not  #1 g_59(Y[58], E[58]);
  not  #1 g_60(Y[59], E[59]);
  not  #1 g_61(Y[60], E[60]);
  not  #1 g_62(Y[61], E[61]);
  not  #1 g_63(Y[62], E[62]);
  not  #1 g_64(Y[63], E[63]);

  and  #12 g_end(eq, Y[0], Y[1], Y[2],Y[3], Y[4], Y[5], Y[6],
          Y[7], Y[8], Y[9], Y[10], Y[11], Y[12], Y[13], Y[14], Y[15], Y[16],
          Y[17], Y[18], Y[19], Y[20], Y[21], Y[22], Y[23], Y[24], Y[25], Y[26], Y[27], Y[28], Y[29], Y[30],
          Y[31], Y[32], Y[33], Y[34], Y[35], Y[36], Y[37], Y[38], Y[39], Y[40], Y[41], Y[42], Y[43], Y[44],
          Y[45], Y[46], Y[47], Y[48], Y[49], Y[50], Y[51], Y[52], Y[53], Y[54], Y[55], Y[56], Y[57],
          Y[58], Y[59], Y[60], Y[61], Y[62], Y[63]); 

endmodule


//and
module and1(A, B, Y);
  input wire A,B;
  output wire Y;

  wire E1;

  nand #1 g1(E1,A,B);
  nand #1 g2(Y,E1);
  
endmodule

module and64bit (A, B, Y);
  input wire [63:0]A;
  input wire[63:0]B;
  output wire [63:0]Y;

  wire E1;

  and1 #2 g1(A[0], B[0], Y[0]);
  and1 #2 g2(A[1], B[1], Y[1]);
  and1 #2 g3(A[2], B[2], Y[2]);
  and1 #2 g4(A[3], B[3], Y[3]);
  and1 #2 g5(A[4], B[4], Y[4]);
  and1 #2 g6(A[5], B[5], Y[5]);
  and1 #2 g7(A[6], B[6], Y[6]);
  and1 #2 g8(A[7], B[7], Y[7]);
  and1 #2 g9(A[8], B[8], Y[8]);
  and1 #2 g10(A[9], B[9], Y[9]);
  and1 #2 g11(A[10], B[10], Y[10]);
  and1 #2 g12(A[11], B[11], Y[11]);
  and1 #2 g13(A[12], B[12], Y[12]);
  and1 #2 g14(A[13], B[13], Y[13]);
  and1 #2 g15(A[14], B[14], Y[14]);
  and1 #2 g16(A[15], B[15], Y[15]);
  and1 #2 g17(A[16], B[16], Y[16]);
  and1 #2 g18(A[17], B[17], Y[17]);
  and1 #2 g19(A[18], B[18], Y[18]);
  and1 #2 g20(A[19], B[19], Y[19]);
  and1 #2 g21(A[20], B[20], Y[20]);
  and1 #2 g22(A[21], B[21], Y[21]);
  and1 #2 g23(A[22], B[22], Y[22]);
  and1 #2 g24(A[23], B[23], Y[23]);
  and1 #2 g25(A[24], B[24], Y[24]);
  and1 #2 g26(A[25], B[25], Y[25]);
  and1 #2 g27(A[26], B[26], Y[26]);
  and1 #2 g28(A[27], B[27], Y[27]);
  and1 #2 g29(A[28], B[28], Y[28]);
  and1 #2 g30(A[29], B[29], Y[29]);
  and1 #2 g31(A[30], B[30], Y[30]);
  and1 #2 g32(A[31], B[31], Y[31]);
  and1 #2 g33(A[32], B[32], Y[32]);
  and1 #2 g34(A[33], B[33], Y[33]);
  and1 #2 g35(A[34], B[34], Y[34]);
  and1 #2 g36(A[35], B[35], Y[35]);
  and1 #2 g37(A[36], B[36], Y[36]);
  and1 #2 g38(A[37], B[37], Y[37]);
  and1 #2 g39(A[38], B[38], Y[38]);
  and1 #2 g40(A[39], B[39], Y[39]);
  and1 #2 g41(A[40], B[40], Y[40]);
  and1 #2 g42(A[41], B[41], Y[41]);
  and1 #2 g43(A[42], B[42], Y[42]);
  and1 #2 g44(A[43], B[43], Y[43]);
  and1 #2 g45(A[44], B[44], Y[44]);
  and1 #2 g46(A[45], B[45], Y[45]);
  and1 #2 g47(A[46], B[46], Y[46]);
  and1 #2 g48(A[47], B[47], Y[47]);
  and1 #2 g49(A[48], B[48], Y[48]);
  and1 #2 g50(A[49], B[49], Y[49]);
  and1 #2 g51(A[50], B[50], Y[50]);
  and1 #2 g52(A[51], B[51], Y[51]);
  and1 #2 g53(A[52], B[52], Y[52]);
  and1 #2 g54(A[53], B[53], Y[53]);
  and1 #2 g55(A[54], B[54], Y[54]);
  and1 #2 g56(A[55], B[55], Y[55]);
  and1 #2 g57(A[56], B[56], Y[56]);
  and1 #2 g58(A[57], B[57], Y[57]);
  and1 #2 g59(A[58], B[58], Y[58]);
  and1 #2 g60(A[59], B[59], Y[59]);
  and1 #2 g61(A[60], B[60], Y[60]);
  and1 #2 g62(A[61], B[61], Y[61]);
  and1 #2 g63(A[62], B[62], Y[62]);
  and1 #2 g64(A[63], B[63], Y[63]);

endmodule



//adder
module rippleadder (A, B, Cin, S, Cout);
  input wire [63:0]A;
  input wire [63:0]B;
  input wire Cin;
  output wire [63:0]S;
  output wire Cout;

  wire [62:0]E;

  fulladder #6 g1(A[0], B[0], Cin, S[0], E[0]);
  fulladder #6 g2(A[1], B[1], E[0], S[1], E[1]);
  fulladder #6 g3(A[2], B[2], E[1], S[2], E[2]);
  fulladder #6 g4(A[3], B[2], E[2], S[3], E[3]);
  fulladder #6 g5(A[4], B[4], E[3], S[4], E[4]);
  fulladder #6 g6(A[5], B[5], E[4], S[5], E[5]);
  fulladder #6 g7(A[6], B[6], E[5], S[6], E[6]);
  fulladder #6 g8(A[7], B[7], E[6], S[7], E[7]);
  fulladder #6 g9(A[8], B[8], E[7], S[8], E[8]);
  fulladder #6 g10(A[9], B[9], E[8], S[9], E[9]);
  fulladder #6 g11(A[10], B[10], E[9], S[10], E[10]);
  fulladder #6 g12(A[11], B[11], E[10], S[11], E[11]);
  fulladder #6 g13(A[12], B[12], E[11], S[12], E[12]);
  fulladder #6 g14(A[13], B[13], E[12], S[13], E[13]);
  fulladder #6 g15(A[14], B[14], E[13], S[14], E[14]);
  fulladder #6 g16(A[15], B[15], E[14], S[15], E[15]);
  fulladder #6 g17(A[16], B[16], E[15], S[16], E[16]);
  fulladder #6 g18(A[17], B[17], E[16], S[17], E[17]);
  fulladder #6 g19(A[18], B[18], E[17], S[18], E[18]);
  fulladder #6 g20(A[19], B[19], E[18], S[19], E[19]);
  fulladder #6 g21(A[20], B[20], E[19], S[20], E[20]);
  fulladder #6 g22(A[21], B[21], E[20], S[21], E[21]);
  fulladder #6 g23(A[22], B[22], E[21], S[22], E[22]);
  fulladder #6 g24(A[23], B[23], E[22], S[23], E[23]);
  fulladder #6 g25(A[24], B[24], E[23], S[24], E[24]);
  fulladder #6 g26(A[25], B[25], E[24], S[25], E[25]);
  fulladder #6 g27(A[26], B[26], E[25], S[26], E[26]);
  fulladder #6 g28(A[27], B[27], E[26], S[27], E[27]);
  fulladder #6 g29(A[28], B[28], E[27], S[28], E[28]);
  fulladder #6 g30(A[29], B[29], E[28], S[29], E[29]);
  fulladder #6 g31(A[30], B[30], E[29], S[30], E[30]);
  fulladder #6 g32(A[31], B[31], E[30], S[31], E[31]);
  fulladder #6 g33(A[32], B[32], E[31], S[32], E[32]);
  fulladder #6 g34(A[33], B[33], E[32], S[33], E[33]);
  fulladder #6 g35(A[34], B[34], E[33], S[34], E[34]);
  fulladder #6 g36(A[35], B[35], E[34], S[35], E[35]);
  fulladder #6 g37(A[36], B[36], E[35], S[36], E[36]);
  fulladder #6 g38(A[37], B[37], E[36], S[37], E[37]);
  fulladder #6 g39(A[38], B[38], E[37], S[38], E[38]);
  fulladder #6 g40(A[39], B[39], E[38], S[39], E[39]);
  fulladder #6 g41(A[40], B[40], E[39], S[40], E[40]);
  fulladder #6 g42(A[41], B[41], E[40], S[41], E[41]);
  fulladder #6 g43(A[42], B[42], E[41], S[42], E[42]);
  fulladder #6 g44(A[43], B[42], E[42], S[43], E[43]);
  fulladder #6 g45(A[44], B[44], E[43], S[44], E[44]);
  fulladder #6 g46(A[45], B[45], E[44], S[45], E[45]);
  fulladder #6 g47(A[46], B[46], E[45], S[46], E[46]);
  fulladder #6 g48(A[47], B[47], E[46], S[47], E[47]);
  fulladder #6 g49(A[48], B[48], E[47], S[48], E[48]);
  fulladder #6 g50(A[49], B[49], E[48], S[49], E[49]);
  fulladder #6 g51(A[50], B[50], E[49], S[50], E[50]);
  fulladder #6 g52(A[51], B[51], E[50], S[51], E[51]);
  fulladder #6 g53(A[52], B[52], E[51], S[52], E[52]);
  fulladder #6 g54(A[53], B[52], E[52], S[53], E[53]);
  fulladder #6 g55(A[54], B[54], E[53], S[54], E[54]);
  fulladder #6 g56(A[55], B[55], E[54], S[55], E[55]);
  fulladder #6 g57(A[56], B[56], E[55], S[56], E[56]);
  fulladder #6 g58(A[57], B[57], E[56], S[57], E[57]);
  fulladder #6 g59(A[58], B[58], E[57], S[58], E[58]);
  fulladder #6 g60(A[59], B[59], E[58], S[59], E[59]);
  fulladder #6 g61(A[60], B[60], E[59], S[60], E[60]);
  fulladder #6 g62(A[61], B[61], E[60], S[61], E[61]);
  fulladder #6 g63(A[62], B[62], E[61], S[62], E[62]);
  fulladder #6 g64(A[63], B[63], E[62], S[63], Cout);
endmodule

module rippleadder32 (A, B, S, Cout);
input wire [31:0]A;
input wire [31:0]B;
output wire [31:0]S;
output wire Cout;
wire Cin;
wire [31:0]E;
  
  assign Cin = 0;
  fulladder #6 g1(A[0], B[0], Cin, S[0], E[0]);
  fulladder #6 g2(A[1], B[1], E[0], S[1], E[1]);
  fulladder #6 g3(A[2], B[2], E[1], S[2], E[2]);
  fulladder #6 g4(A[3], B[2], E[2], S[3], E[3]);
  fulladder #6 g5(A[4], B[4], E[3], S[4], E[4]);
  fulladder #6 g6(A[5], B[5], E[4], S[5], E[5]);
  fulladder #6 g7(A[6], B[6], E[5], S[6], E[6]);
  fulladder #6 g8(A[7], B[7], E[6], S[7], E[7]);
  fulladder #6 g9(A[8], B[8], E[7], S[8], E[8]);
  fulladder #6 g10(A[9], B[9], E[8], S[9], E[9]);
  fulladder #6 g11(A[10], B[10], E[9], S[10], E[10]);
  fulladder #6 g12(A[11], B[11], E[10], S[11], E[11]);
  fulladder #6 g13(A[12], B[12], E[11], S[12], E[12]);
  fulladder #6 g14(A[13], B[13], E[12], S[13], E[13]);
  fulladder #6 g15(A[14], B[14], E[13], S[14], E[14]);
  fulladder #6 g16(A[15], B[15], E[14], S[15], E[15]);
  fulladder #6 g17(A[16], B[16], E[15], S[16], E[16]);
  fulladder #6 g18(A[17], B[17], E[16], S[17], E[17]);
  fulladder #6 g19(A[18], B[18], E[17], S[18], E[18]);
  fulladder #6 g20(A[19], B[19], E[18], S[19], E[19]);
  fulladder #6 g21(A[20], B[20], E[19], S[20], E[20]);
  fulladder #6 g22(A[21], B[21], E[20], S[21], E[21]);
  fulladder #6 g23(A[22], B[22], E[21], S[22], E[22]);
  fulladder #6 g24(A[23], B[23], E[22], S[23], E[23]);
  fulladder #6 g25(A[24], B[24], E[23], S[24], E[24]);
  fulladder #6 g26(A[25], B[25], E[24], S[25], E[25]);
  fulladder #6 g27(A[26], B[26], E[25], S[26], E[26]);
  fulladder #6 g28(A[27], B[27], E[26], S[27], E[27]);
  fulladder #6 g29(A[28], B[28], E[27], S[28], E[28]);
  fulladder #6 g30(A[29], B[29], E[28], S[29], E[29]);
  fulladder #6 g31(A[30], B[30], E[29], S[30], E[30]);
  fulladder #6 g32(A[31], B[31], E[30], S[31], E[31]);
endmodule

module fulladder (A, B, Cin, S, Cout);
  input wire A, B, Cin;
  output wire S, Cout;

  wire E1, E2, E3;

  xor #3 g1(E1, A, B);
  and #2 g2(E2, A, B);
  xor #3 g3(S, E1, Cin);
  and #2 g4(E3, E1, Cin);
  or #2 g5(Cout, E3, E2);

endmodule
