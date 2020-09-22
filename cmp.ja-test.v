module cmpja_test();
reg clk, rst;
wire jump;
wire [31:0]instr;
wire [31:0]E1;
wire [15:0] sign;
wire [4:0]rd;
wire [4:0]rt;
wire [5:0]op;
wire [63:0]out1;
wire [63:0]out2;
wire [63:0]signout;
wire mux1, mux2, mux3, Regenable, ALUenable, Dataenable, datarw;
wire [63:0]dataOut;
wire eq;
wire [4:0]mux1out;
wire [63:0]mux2out;
wire [63:0]mux3out;
wire [63:0]ALUout;


thirdcpu DUT(clk, rst);
initial
begin
$dumpfile("mytest.vcd");
$dumpvars;
clk <= 0; rst <= 1;
end
always #50 clk=~clk;

initial
begin
        #100
        clk <= 0;
        rst <= 0;

        #1500
        $finish();
end

endmodule
