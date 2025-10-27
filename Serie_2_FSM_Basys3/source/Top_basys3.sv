`timescale 1ns / 1ps

module Top_basys3(
  input  logic        clk,      // 100 MHz Basys3
  input  logic        btnC,     // reset global
  input  logic        btnU,     // alternar vista (FSM <-> Clock)
  input  logic [2:0]  sw,       // sw[0]=iT, sw[1]=iM, sw[2]=C

  // 7-seg
  output logic [3:0]  an,       // anodos activos en bajo
  output logic [6:0]  seg,      // segmentos a-g activos en bajo
  output logic        dp        // punto decimal (activo en bajo)
);


  logic clk_slow;
  clk_psc clock_div (
    .clk(clk),
    .clk_scaled(clk_slow)   // ~0.7..1.5 Hz (ajustable dentro)
  );


  logic T1, V1;
  logic [1:0] D1, TAL2;

  Top_FSM sistema (
    .clk(clk_slow),     // FSM con reloj dividido
    .R(btnC),
    .iT(sw[0]),
    .iM(sw[1]),
    .C(sw[2]),
    .T1(T1),
    .V1(V1),
    .D1(D1),
    .TAL2(TAL2)
  );


  logic [3:0] hora_d, hora_u, min_d, min_u;

  clock reloj_inst(
    .clk   (clk),      // usa el clock rápido (100MHz)
    .reset (btnC),
    .hora_d(hora_d),
    .hora_u(hora_u),
    .min_d (min_d),
    .min_u (min_u)
  );


  logic btn_db, btn_pulse;
  logic show_clock = 1'b0;         // 0=FSM (default), 1=Clock

  debouncer db(.clk(clk), .btn_raw(btnU), .btn_stable(btn_db));
  one_pulse  op(.clk(clk), .din(btn_db), .pulse(btn_pulse));

  always_ff @(posedge clk) begin
    if (btnC) show_clock <= 1'b0;  // reset → FSM
    else if (btn_pulse) show_clock <= ~show_clock;
  end


  logic [3:0] d3, d2, d1_bcd, d0_bcd;

  always_comb begin
    if (show_clock) begin
      // Vista reloj HH:MM
      d3 = hora_d;            // H tens
      d2 = hora_u;            // H ones
      d1_bcd = min_d;         // M tens
      d0_bcd = min_u;         // M ones
    end else begin
      // Vista FSM
      d3 = {2'b00, TAL2};     // 0..3 (estado talanquera)
      d2 = 4'hF;              // blank
      d1_bcd = 4'hF;          // blank
      d0_bcd = {2'b00, D1};   // 0..2 (contador monedas)
    end
  end

  display_7segments disp(
    .clk(clk),
    .d3 (d3),
    .d2 (d2),
    .d1 (d1_bcd),
    .d0 (d0_bcd),
    .enabled(an),
    .ag(seg)
  );

  assign dp = 1'b1; // apaga el punto decimal (ánodo común)

endmodule
