`timescale 1ns / 1ps
module debouncer(
  input  logic clk,
  input  logic btn_raw,
  output logic btn_stable
);
  logic [15:0] cnt;
  logic sync0, sync1;

  always_ff @(posedge clk) begin
    sync0 <= btn_raw;
    sync1 <= sync0;

    if (sync1) begin
      if (&cnt) btn_stable <= 1'b1;
      else      cnt <= cnt + 1;
    end else begin
      if (&cnt) btn_stable <= 1'b0;
      else      cnt <= cnt + 1;
    end
  end
endmodule
