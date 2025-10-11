`timescale 1ns / 1ps

module M2_Mealy(
    input  logic clk,
    input  logic R,    // reset síncrono
    input  logic T,    // ticket presente
    input  logic V,    // ticket validado
    input  logic C,    // sensor carro
    output logic [1:0] TAL
);

    // Estados
    typedef enum logic [1:0] {S0, S1, S2, S3} statetype;
    statetype state, nextstate;

    // Salidas de la talanquera
    typedef enum logic [1:0] {DOWN, UP_START, UP, DOWN_START} outtype;
    outtype tal;

    // Registro de estado (reset síncrono)
    always_ff @(posedge clk) begin
        if (R)
            state <= S0;
        else
            state <= nextstate;
    end

    // next state logic
    always_comb begin
        nextstate = state; // valor por defecto
        case (state)
            S0: nextstate = (T & V & C) ? S1 : S0;
            S1: nextstate = (T & V & C) ? S2 : S1;
            S2: nextstate = (!C) ? S3 : S2;
            S3: nextstate = (!C) ? S0 : S3;
            default: nextstate = S0;
        endcase
    end

    // output logic
    always_comb begin
        tal = DOWN; // valor por defecto
        case (state)
            S0: tal = (T & V & C) ? UP_START : DOWN;
            S1: tal = (T & V & C) ? UP : UP_START;
            S2: tal = (C) ? UP : DOWN_START;
            S3: tal = (C) ? DOWN : DOWN_START;
            default: tal = DOWN;
        endcase
    end

    // Conectar enum de salida al puerto binario
    assign TAL = tal;

endmodule
