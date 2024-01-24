`timescale 1ns / 1ps
module sdram_ctrl_tb();
  localparam RowWidth    = 12;
  localparam ColWidth    = 8;
  localparam BankWidth   = 2;
  localparam AddrWidth   = BankWidth + ColWidth + RowWidth;
  localparam DataWidth   = 16;
  localparam ClockFreq   = 133_000_000;
  localparam ClockPeriod = 1_000_000_000 / ClockFreq;

  localparam BurstLength = 1;

  logic i_sys_clk, i_dram_clk;
  logic i_rst_n;
  logic i_wr_req, i_rd_req, o_rd_rdy;
  
  logic [AddrWidth-1:0] i_wr_addr;
  logic [AddrWidth-1:0] i_rd_addr;
  logic [DataWidth-1:0] i_wr_data [BurstLength];
  logic [DataWidth-1:0] o_rd_data [BurstLength];

  logic [DataWidth-1:0] read_data;

  sdram_ctrl #(
    .ClockFreq(ClockFreq),
    .BurstLength(BurstLength)
  ) uut (
    .i_sys_clk,   
    .i_dram_clk,  
    .i_rst_n,     
    .i_wr_req,
    .i_wr_addr,
    .i_wr_data,
    .i_rd_req,
    .i_rd_addr,
    .o_rd_data,
    .o_rd_rdy,
    .o_dram_addr(), 
    .io_dram_data(), 
    .o_dram_ba_0(), 
    .o_dram_ba_1(), 
    .o_dram_ldqm(), 
    .o_dram_udqm(), 
    .o_dram_we_n(), 
    .o_dram_cas_n(),
    .o_dram_ras_n(),
    .o_dram_cs_n(), 
    .o_dram_clk(),  
    .o_dram_cke()   
  );

    // Clock Gen
  initial i_sys_clk = 1'b0;
  always #(ClockPeriod / 2) i_sys_clk = ~i_sys_clk;
  assign i_dram_clk = i_sys_clk; // Assume both running at DRAM Clock speed

  task write_sdram (
    input logic [DataWidth-1:0] i_data,
    input logic [AddrWidth-1:0] i_addr
  );
    @(posedge i_dram_clk); // Sync back to clock
    i_wr_req     <= '1;
    i_wr_data[0] <= i_data;
    i_wr_addr    <= i_addr;
    @(posedge i_dram_clk);
    i_wr_req     <= '0; // Deassert request
    @(posedge i_dram_clk);
  endtask

  task read_sdram (
    input  logic [AddrWidth-1:0] i_addr,
    output logic [DataWidth-1:0] o_data
  );
    i_rd_req  <= '1;
    i_rd_addr <= i_addr;
    @(posedge i_dram_clk);
    i_rd_req  <= '0; // Deassert request
    @(posedge i_dram_clk);
    wait(o_rd_rdy);
    o_data    <= o_rd_data[0];
    @(posedge i_dram_clk);
  endtask

  initial begin
    $dumpfile("sdram_ctrl_tb.vcd"); // Initialize VCD dump
    $dumpvars(0, sdram_ctrl_tb);    // Dump all variables in this module
    
    // Initial Signal Stimuli
    i_rst_n      <= '0; // Assert reset
    i_wr_req     <= '0;
    i_wr_data[0] <= '0;
    i_wr_addr    <= '0;
    i_rd_req     <= '0;
    i_rd_addr    <= '0;
    repeat (10) @(posedge i_dram_clk);
    i_rst_n      <= '1;

    #(201000); // Wait for init to finish
    write_sdram($random, {2'd0, 8'd5, 12'd13});
    repeat (15) @(posedge i_dram_clk);
    write_sdram($random, {2'd0, 8'd5, 12'd13});
    repeat (100) @(posedge i_dram_clk);

    read_sdram({2'd0, 8'd5, 12'd13}, read_data); // Bank 0
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd1, 8'd5, 12'd13}, read_data); // Bank 1, new bank, should be no precharge, just ACT
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd9, 12'd13}, read_data); // Bank 0, same row, different col, should be NO precharge
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd15, 12'd13}, read_data); // Bank 0, same row, different col, should be NO precharge
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd15, 12'd15}, read_data); // Bank 0, different row, should be precharge and ACT

    repeat (2000) @(posedge i_dram_clk);
    // By now, there should've been a refresh. Let's make sure that a read to the previously open row does NOT skip a precharge
    read_sdram({2'd0, 8'd15, 12'd15}, read_data); // Bank 0, same row, should be ACT & precharge though, as it's been closed during our all bank precharge in refresh cycle
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd35, 12'd20}, read_data); // Bank 0, different row, should be precharge
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd1, 12'd20}, read_data); // Bank 0, different col, should be NO precharge
    repeat (15) @(posedge i_dram_clk);
    read_sdram({2'd0, 8'd1, 12'd21}, read_data); // Bank 0, different row, should be precharge

    repeat(50) @(posedge i_dram_clk);

    repeat(4000) @(posedge i_dram_clk);

    $display("sdram_ctrl testbench complete");
    $finish;
  end

endmodule
