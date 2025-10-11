`timescale 1ns / 1ps

module Top_fpga(
    input  logic clk,        // reloj físico
    input  logic btnC,       // reset
    input  logic [2:0] sw,   // sw[0] = iT, sw[1] = iM, sw[2] = C
    output logic [6:0] led   // leds 0-6
);
    logic clk_slow;
    logic T1, V1;
    logic [1:0] D1, TAL2;

    clk_psc clock_div (
        .clk(clk),
        .clk_scaled(clk_slow)
    );

    Top sistema (
        .clk(clk_slow),
        .R(btnC),
        .iT(sw[0]),
        .iM(sw[1]),
        .C(sw[2]),
        .T1(T1),
        .V1(V1),
        .D1(D1),
        .TAL2(TAL2)
    );

    // LD6 = clk lento
    // LD0-1 = TAL2 (talanquera)
    // LD2-3 = D1   (dinero)
    // LD4   = T1   (ticket detectado)
    // LD5   = V1   (ticket validado)
    assign led[6]   = clk_slow;
    assign led[1:0] = TAL2;
    assign led[3:2] = D1;
    assign led[4]   = T1;
    assign led[5]   = V1;

endmodule
