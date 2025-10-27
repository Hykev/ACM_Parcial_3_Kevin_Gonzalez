`timescale 1ns / 1ps

module clock(
    input  logic       clk, reset,
    output logic [3:0] hora_d, hora_u, min_d, min_u
);

    logic [26:0] contador = '0;   // 27 bits alcanzan 100M-1
    logic [5:0]  segundos = '0;   // 0..59
    logic [3:0]  mins_u   = '0;
    logic [3:0]  mins_d   = '0;
    logic [3:0]  horas_u  = '0;
    logic [3:0]  horas_d  = '0;

    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            contador <= '0; segundos <= '0;
            mins_u <= '0; mins_d <= '0;
            horas_u <= '0; horas_d <= '0;
        end else begin
            // Tick 1s 100MHz
            if (contador == 100_000_000 - 1) begin
                contador <= '0;
                if (segundos == 59) begin
                    segundos <= 0;
                    // ++ minutos
                    if (mins_u == 9) begin
                        mins_u <= 0;
                        if (mins_d == 5) begin
                            mins_d <= 0;
                            // ++ horas (24h)
                            if (horas_d == 2 && horas_u == 3) begin
                                horas_d <= 0; horas_u <= 0; // 23 -> 00
                            end else if (horas_u == 9) begin
                                horas_u <= 0; horas_d <= horas_d + 1;
                            end else begin
                                horas_u <= horas_u + 1;
                            end
                        end else begin
                            mins_d <= mins_d + 1;
                        end
                    end else begin
                        mins_u <= mins_u + 1;
                    end
                end else begin
                    segundos <= segundos + 1;
                end
            end else begin
                contador <= contador + 1;
            end
        end
    end

    assign min_u  = mins_u;
    assign min_d  = mins_d;
    assign hora_u = horas_u;
    assign hora_d = horas_d;

endmodule
