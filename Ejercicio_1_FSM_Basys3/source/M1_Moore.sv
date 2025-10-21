`timescale 1ns / 1ps

// I/O
module M1_Moore(input logic clk,
                input logic iT,
                input logic iM,
                input logic R,
                output logic T, V,
                output  logic [1:0] D);
    
    typedef enum logic [2:0] {S0, S1, S2, S3} statetype;
    statetype state, nextstate;
        
    // flip-flop
    always_ff @(posedge clk)
        if (R) state <= S0;
        else state <= nextstate;
    
    // next state logic
always_comb begin
    nextstate = state; // default
    case (state)
        S0: begin
            if (iT) nextstate = S1;
            else nextstate = S0;
        end
        S1: begin
            if (iM) nextstate = S2;
            else nextstate = S1;
        end
        S2: begin
            if (iM) nextstate = S3;
            else nextstate = S2;
        end
        S3: begin
            if (R) nextstate = S0;
            else nextstate = S3;
        end
        default: nextstate = S0;
    endcase
end
    
    always_comb begin
        T = 0; V = 0; D = 2'b00; // valores por defecto
        case (state)
            S0 : begin T = 0; V = 0; D = 'b00; end
            S1 : begin T = 1; V = 0; D = 'b00; end
            S2 : begin T = 1; V = 0; D = 'b01; end
            S3 : begin T = 1; V = 1; D = 'b10; end
            default : begin T = 0; V = 0; D = 'b00; end
        endcase
    end

endmodule