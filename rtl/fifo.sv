// Synchronous FIFO (First-in First-Out) buffer
`timescale 1ns / 1ps
module fifo #(
  parameter  DataWidth = 8,
  parameter  Depth     = 8,
  localparam PtrWidth  = $clog2(Depth)
)(
  input  logic                 i_rd_clk,
  input  logic                 i_wr_clk,
  input  logic                 i_rst_n,
  input  logic [DataWidth-1:0] i_wr_data,
  input  logic                 i_wr_en,
  input  logic                 i_rd_en,
  output logic [DataWidth-1:0] o_rd_data,
  output logic                 o_full,
  output logic                 o_empty
);

  logic [DataWidth-1:0] mem_rd_data;
  logic [PtrWidth-1:0]  wr_addr, rd_addr;

  assign o_rd_data = mem_rd_data;

  fifo_mem #(
    .DataWidth(DataWidth),
    .Depth(Depth)
  ) fifo_mem (
    .i_rd_clk,
    .i_wr_clk,
    .i_wr_data,
    .i_wr_addr(wr_addr),
    .i_wr_en,
    .i_rd_addr(rd_addr),
    .o_rd_data(mem_rd_data)
  );

  fifo_ctrl #(
    .DataWidth(DataWidth),
    .Depth(Depth)
  ) fifo_ctrl (
    .i_rd_clk,
    .i_wr_clk,
    .i_rst_n,
    .i_wr_en,
    .i_rd_en,
    .o_rd_addr(rd_addr),
    .o_wr_addr(wr_addr),
    .o_full,
    .o_empty
  );
  
endmodule

