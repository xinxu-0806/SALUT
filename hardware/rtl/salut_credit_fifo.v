`default_nettype none

// DIAGNOSTIC PROTOTYPE ONLY; not installed in any XO/checkpoint.
// Single clock, FWFT, conservative full/empty. Exactly REAL_DEPTH entries.
// Both sequence mirrors are registered twice. Thus a consumer combinational
// read decision cannot feed the producer's full logic in the same cycle.
// This is NOT an asynchronous CDC primitive and needs normal 5 ns timing on
// every path; no false-path or multicycle exception is permitted.
module salut_credit_fifo #(
  parameter DATA_WIDTH = 41,
  parameter ADDR_WIDTH = 2,
  parameter DEPTH = 4
) (
  input wire clk,
  input wire reset,
  output wire if_full_n,
  input wire if_write_ce,
  input wire if_write,
  input wire [DATA_WIDTH-1:0] if_din,
  output wire if_empty_n,
  input wire if_read_ce,
  input wire if_read,
  output wire [DATA_WIDTH-1:0] if_dout
);
  localparam integer REAL_DEPTH = DEPTH < 4 ? 4 : DEPTH;
  localparam integer PTR_WIDTH = $clog2(REAL_DEPTH);
  localparam integer SEQ_WIDTH = PTR_WIDTH + 1;
  // The prototype intentionally supports powers of two only, failing closed
  // for any other depth; the current publication FIFOs have depth four.
  // synthesis translate_off
  initial begin
    if (DATA_WIDTH < 1 || (REAL_DEPTH & (REAL_DEPTH-1)) != 0)
      $fatal(1, "CREDIT_FIFO_POWER_OF_TWO_DEPTH_AND_POSITIVE_WIDTH_REQUIRED");
  end
  // synthesis translate_on

  (* ram_style = "distributed" *)
  reg [DATA_WIDTH-1:0] mem [0:REAL_DEPTH-1];
  wire [SEQ_WIDTH-1:0] write_sequence;
  wire [SEQ_WIDTH-1:0] read_sequence;
  wire do_write = if_write && if_write_ce && if_full_n;
  wire do_read = if_read && if_read_ce && if_empty_n;

  generate begin : producer_control
    reg [SEQ_WIDTH-1:0] sequence_reg;
    (* shreg_extract = "no" *) reg [SEQ_WIDTH-1:0] read_seen_1;
    (* shreg_extract = "no" *) reg [SEQ_WIDTH-1:0] read_seen_2;
    wire [SEQ_WIDTH-1:0] outstanding = sequence_reg - read_seen_2;
    assign write_sequence = sequence_reg;
    assign if_full_n = !reset && outstanding < REAL_DEPTH;
    always @(posedge clk) begin
      if (reset) begin
        sequence_reg <= 0;
        read_seen_1 <= 0;
        read_seen_2 <= 0;
      end else begin
        read_seen_1 <= read_sequence;
        read_seen_2 <= read_seen_1;
        if (do_write) sequence_reg <= sequence_reg + 1'b1;
      end
    end
  end endgenerate

  generate begin : consumer_control
    reg [SEQ_WIDTH-1:0] sequence_reg;
    (* shreg_extract = "no" *) reg [SEQ_WIDTH-1:0] write_seen_1;
    (* shreg_extract = "no" *) reg [SEQ_WIDTH-1:0] write_seen_2;
    assign read_sequence = sequence_reg;
    assign if_empty_n = !reset && sequence_reg != write_seen_2;
    always @(posedge clk) begin
      if (reset) begin
        sequence_reg <= 0;
        write_seen_1 <= 0;
        write_seen_2 <= 0;
      end else begin
        write_seen_1 <= write_sequence;
        write_seen_2 <= write_seen_1;
        if (do_read) sequence_reg <= sequence_reg + 1'b1;
      end
    end
  end endgenerate

  assign if_dout = mem[read_sequence[PTR_WIDTH-1:0]];
  always @(posedge clk) begin
    if (do_write)
      mem[write_sequence[PTR_WIDTH-1:0]] <= if_din;
  end
endmodule

`default_nettype wire
