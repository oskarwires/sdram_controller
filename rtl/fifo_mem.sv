// Dual-Port RAM for FIFO, async read and sync write, with no reset
`timescale 1ns / 1ps
module fifo_mem #(
  parameter  DataWidth = 8,
  parameter  Depth     = 8,
  localparam AddrWidth = $clog2(Depth)
)(
  input  logic                 i_clk,
  input  logic                 i_wr_en,
  input  logic [AddrWidth-1:0] i_wr_addr,
  input  logic [DataWidth-1:0] i_wr_data,
  input  logic [AddrWidth-1:0] i_rd_addr,
  output logic [DataWidth-1:0] o_rd_data
);

  logic [DataWidth-1:0] memory [Depth];
  assign o_rd_data = memory[i_rd_addr];
  
  always_ff @(posedge i_clk) begin
    if (i_wr_en) memory[i_wr_addr] <= i_wr_data;
  end
  
endmodule
