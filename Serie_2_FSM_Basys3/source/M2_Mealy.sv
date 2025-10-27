`timescale 1ns / 1ps

module M2_Mealy(
    input  logic clk,
    input  logic R,    // reset síncrono
    input  logic T,    // ticket presente
    input  logic V,    // ticket validado
    input  logic C,    // sensor carro (1 = presente)
    output logic [1:0] TAL
);
    typedef enum logic [1:0] {S0, S1, S2, S3} statetype;
    statetype state, nextstate;

    typedef enum logic [1:0] {DOWN, UP_START, UP, DOWN_START} outtype;
    outtype tal;

    // Estado
    always_ff @(posedge clk) begin
        if (R) state <= S0;
        else   state <= nextstate;
    end

    // Next state
    always_comb begin
        nextstate = state;
        case (state)
            S0: if (T & V & C) nextstate = S1;
            S1: if (T & V & C) nextstate = S2;
            S2: if (!C)        nextstate = S3;
            S3: if (!C)        nextstate = S0;
        endcase
    end

    // Salidas (Mealy)
    always_comb begin
        tal = DOWN;
        case (state)
            S0: tal = (T & V & C) ? UP_START : DOWN;
            S1: tal = (T & V & C) ? UP       : UP_START;
            S2: tal = (!C)        ? DOWN_START : UP;
            S3: tal = (!C)        ? DOWN       : DOWN_START;
        endcase
    end

    assign TAL = tal;
endmodule
