// Isolated full-RTL derivative, not production or timing acceptance.
// Explicitly selected by 32 seg_task_<n> instances, never by width-only policy.
`default_nettype none
module salut_dispatch_fifo #(
  parameter integer DATA_WIDTH = 385,
  parameter integer ADDR_WIDTH = 3,
  parameter integer DEPTH = 8
) (
  input wire clk, input wire reset,
  output wire if_full_n,
  input wire if_write_ce, input wire if_write,
  input wire [DATA_WIDTH-1:0] if_din,
  output wire if_empty_n,
  input wire if_read_ce, input wire if_read,
  output wire [DATA_WIDTH-1:0] if_dout
);
  fifo_bram_exact_capacity #(
    .DATA_WIDTH(DATA_WIDTH), .ADDR_WIDTH(ADDR_WIDTH), .DEPTH(DEPTH)
  ) exact_storage (
    .clk(clk), .reset(reset),
    .if_full_n(if_full_n), .if_write_ce(if_write_ce),
    .if_write(if_write), .if_din(if_din),
    .if_empty_n(if_empty_n), .if_read_ce(if_read_ce),
    .if_read(if_read), .if_dout(if_dout)
  );
endmodule
`default_nettype wire
