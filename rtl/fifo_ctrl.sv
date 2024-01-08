// FIFO Controller module
`timescale 1ns / 1ps
module fifo_ctrl #(
  parameter  DataWidth   = 8,
  parameter  Depth       = 8,
  localparam PtrWidth    = $clog2(Depth)
)(
  input  logic                i_clk,
  input  logic                i_rst_n,
  input  logic                i_wr_en,
  input  logic                i_rd_en,
  output logic [PtrWidth-1:0] o_wr_addr,
  output logic [PtrWidth-1:0] o_rd_addr,
  output logic                o_full,
  output logic                o_empty
);

  logic [DataWidth-1:0] buffer[Depth];
  logic [PtrWidth:0]    wr_ptr, wr_ptr_next;
  logic [PtrWidth:0]    rd_ptr, rd_ptr_next;                 

  always_ff @(posedge i_clk) begin
    if (!i_rst_n) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
    end else begin
      wr_ptr <= wr_ptr_next;
      rd_ptr <= rd_ptr_next;
    end
  end

  always_comb begin
    if (i_wr_en) wr_ptr_next = wr_ptr + 1'b1;
    else         wr_ptr_next = wr_ptr;

    if (i_rd_en) rd_ptr_next = rd_ptr + 1'b1;
    else         rd_ptr_next = rd_ptr;
  end
  
  assign o_rd_addr = rd_ptr[PtrWidth-1:0];
  assign o_wr_addr = wr_ptr[PtrWidth-1:0];

  assign o_full  = ((wr_ptr[PtrWidth] != rd_ptr[PtrWidth]) && (wr_ptr[PtrWidth-1:0] == rd_ptr[PtrWidth-1:0]));
  assign o_empty = ((wr_ptr[PtrWidth] == rd_ptr[PtrWidth]) && (wr_ptr[PtrWidth-1:0] == rd_ptr[PtrWidth-1:0]));
  
endmodule

