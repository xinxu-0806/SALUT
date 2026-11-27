// SALUT control-infrastructure pipelining for the SLR-partitioned top level.
// None of these modules touches stream data or compute. They only add flops on the
// kernel reset, quasi-static AXI-Lite scalars, sticky task ap_start, and the narrow
// META AXI4 channels, so those nets no longer cross SLRs combinationally.

// One synchronous reset pipeline flop. INVERT=1 turns the active-low input into an
// active-high output. The power-up value is the asserted level, so the kernel stays in
// reset until the platform reset has propagated. Every consumer in the top level sees
// the same number of stages, so all modules leave reset in the same cycle.
module salut_rst_stage #(
  parameter integer INVERT = 0
) (
  input  wire clk,
  input  wire d,
  output wire q
);
  (* keep = "true", shreg_extract = "no" *) reg r = INVERT ? 1'b1 : 1'b0;
  always @(posedge clk) r <= INVERT ? ~d : d;
  assign q = r;
endmodule

// One free-running register stage on an AXI-Lite mmap base offset. The host writes scalars
// at least three AXI-Lite write cycles before ap_start, and each task's ap_start is
// registered at least once more, so two stages settle before any task can sample the value.
module salut_scalar_stage #(
  parameter integer WIDTH = 64
) (
  input  wire             clk,
  input  wire [WIDTH-1:0] d,
  output wire [WIDTH-1:0] q
);
  (* shreg_extract = "no" *) reg [WIDTH-1:0] r = {WIDTH{1'b0}};
  always @(posedge clk) r <= d;
  assign q = r;
endmodule

// One free-running flop on a sticky (detached) task ap_start. The TAPA FSM's per-task
// ap_start registers are logically identical, so without keep synthesis would merge every
// task's pipeline into one flop and recreate a chip-wide high-fanout net. Delaying a sticky
// ap_start only starts the task later; every inter-task channel is a FIFO, and the platform
// reset pulse is far longer than the pipeline, so reset flushes these stages to 0.
module salut_start_stage (
  input  wire clk,
  input  wire d,
  output wire q
);
  (* keep = "true", shreg_extract = "no" *) reg r = 1'b0;
  always @(posedge clk) r <= d;
  assign q = r;
endmodule

// Full-throughput valid/ready register slice (two-entry skid buffer). Both s_ready and
// the m_* outputs are registered. Once m_valid rises, it and m_data hold until m_ready,
// as AXI requires. One instance per AXI channel per stage.
module salut_axi_skid #(
  parameter integer WIDTH = 1
) (
  input  wire             clk,
  input  wire             rst,
  input  wire             s_valid,
  output wire             s_ready,
  input  wire [WIDTH-1:0] s_data,
  output wire             m_valid,
  input  wire             m_ready,
  output wire [WIDTH-1:0] m_data
);
  (* shreg_extract = "no" *) reg [WIDTH-1:0] data_r = {WIDTH{1'b0}};
  (* shreg_extract = "no" *) reg [WIDTH-1:0] skid_r = {WIDTH{1'b0}};
  reg valid_r = 1'b0, skid_valid = 1'b0;
  assign s_ready = ~skid_valid;
  assign m_valid = valid_r;
  assign m_data = data_r;
  always @(posedge clk) begin
    if (rst) begin
      valid_r <= 1'b0;
      skid_valid <= 1'b0;
    end else if (m_ready || !valid_r) begin
      if (skid_valid) begin
        data_r <= skid_r;
        valid_r <= 1'b1;
        skid_valid <= 1'b0;
      end else begin
        valid_r <= s_valid;
        if (s_valid) data_r <= s_data;
      end
    end else if (s_valid && !skid_valid) begin
      skid_r <= s_data;
      skid_valid <= 1'b1;
    end
  end
endmodule
