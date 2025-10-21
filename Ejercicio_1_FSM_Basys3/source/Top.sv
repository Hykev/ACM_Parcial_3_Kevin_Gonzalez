`timescale 1ns / 1ps

module Top(
    input  logic clk,
    input  logic R,
    input  logic iT,   // entrada libre para M1
    input  logic iM,   // entrada libre para M1
    input  logic C,    // entrada libre para M2
    output logic T1, V1,           // salidas de M1
    output logic [1:0] D1,         // salida D de M1
    output logic [1:0] TAL2        // salida talanquera
);
    // Señales internas
    logic V2;

    typedef enum logic [1:0] {DOWN, UP_START, UP, DOWN_START} tal_state_t;
    tal_state_t TAL_text;

    // Instancia de M1
    M1_Moore Maquina_sellado (
        .clk(clk),
        .iT(iT),
        .iM(iM),
        .R(R),
        .T(T1),
        .V(V1),
        .D(D1)
    );

    // Conexion para M2
    assign V2 = V1;

    // Instancia de M2
    M2_Mealy Talanquera (
        .clk(clk),
        .R(R),
        .T(~iT),
        .V(V2),
        .C(C),
        .TAL(TAL2)
    );

    always_comb begin
        case(TAL2)
            2'b00: TAL_text = DOWN;
            2'b01: TAL_text = UP_START;
            2'b10: TAL_text = UP;
            2'b11: TAL_text = DOWN_START;
            default: TAL_text = DOWN;
        endcase
    end

endmodule
