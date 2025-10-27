`timescale 1ns / 1ps

module display_7segments(
    input  logic       clk,         // 100 MHz
    input  logic [3:0] d3, d2, d1, d0,   // dígitos BCD
    output logic [3:0] enabled,     // anodos (activos en bajo)
    output logic [6:0] ag           // segmentos a..g (activos en bajo)
);
    logic [16:0] conteo = '0;  // ~1ms a 100MHz (hasta 100_000)
    logic [1:0]  fsm    = '0;
    logic [3:0]  display_number;

    // Refresh cada ~1ms
    always_ff @(posedge clk) begin
        if (conteo == 100_000 - 1) begin
            conteo <= 0;
            fsm    <= fsm + 1;
        end else begin
            conteo <= conteo + 1;
        end
    end

    // Selección de display (AN0..AN3)
    always_comb begin
        case (fsm)
            2'b00: begin enabled = 4'b1110; display_number = d0; end // AN0
            2'b01: begin enabled = 4'b1101; display_number = d1; end // AN1
            2'b10: begin enabled = 4'b1011; display_number = d2; end // AN2
            2'b11: begin enabled = 4'b0111; display_number = d3; end // AN3
        endcase
    end

    always_comb begin
        unique case (display_number)
            4'd0: ag = 7'b1000000;
            4'd1: ag = 7'b1111001;
            4'd2: ag = 7'b0100100;
            4'd3: ag = 7'b0110000;
            4'd4: ag = 7'b0011001;
            4'd5: ag = 7'b0010010;
            4'd6: ag = 7'b0000010;
            4'd7: ag = 7'b1111000;
            4'd8: ag = 7'b0000000;
            4'd9: ag = 7'b0010000;
            default: ag = 7'b1111111; // off
        endcase
    end
endmodule
