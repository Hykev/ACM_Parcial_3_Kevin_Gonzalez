`timescale 1ns / 1ps

module clk_psc(
    input  logic clk,         // 100 MHz
    output logic clk_scaled
);
    logic [31:0] myreg = 0;

    always_ff @(posedge clk)
        myreg <= myreg + 1;

    assign clk_scaled = myreg[26];
endmodule
