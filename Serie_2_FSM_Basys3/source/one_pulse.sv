`timescale 1ns / 1ps
module one_pulse(
  input  logic clk,
  input  logic din,
  output logic pulse
);
  logic d_ff;
  always_ff @(posedge clk) begin
    pulse <= din & ~d_ff;
    d_ff  <= din;
  end
endmodule
