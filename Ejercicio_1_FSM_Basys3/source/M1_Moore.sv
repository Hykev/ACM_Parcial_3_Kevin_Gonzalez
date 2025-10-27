`timescale 1ns / 1ps

module M1_Moore(
    input  logic clk,
    input  logic iT,
    input  logic iM,
    input  logic R,
    output logic T, V,
    output logic [1:0] D
);
    typedef enum logic [2:0] {S0, S1, S2, S3} statetype;
    statetype state, nextstate;

    // FF de estado
    always_ff @(posedge clk)
        if (R) state <= S0;
        else   state <= nextstate;

    // Next state
    always_comb begin
        nextstate = state;
        case (state)
            S0: if (iT) nextstate = S1;
            S1: if (iM) nextstate = S2;
            S2: if (iM) nextstate = S3;
            S3: if (R)  nextstate = S0;
        endcase
    end

    // Salidas tipo Moore
    always_comb begin
        T=0; V=0; D=2'b00;
        case (state)
            S0: begin T=0; V=0; D=2'b00; end
            S1: begin T=1; V=0; D=2'b00; end
            S2: begin T=1; V=0; D=2'b01; end
            S3: begin T=1; V=1; D=2'b10; end
        endcase
    end
endmodule
