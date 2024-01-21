`timescale 1ns / 1ps
module sdram_ctrl_tb();
  localparam RowWidth    = 12;
  localparam ColWidth    = 8;
  localparam BankWidth   = 2;
  localparam AddrWidth   = BankWidth + ColWidth + RowWidth;
  localparam DataWidth   = 16;
  localparam ClockFreq   = 133_000_000;
  localparam ClockPeriod = 1_000_000_000 / ClockFreq;

  logic i_sys_clk, i_dram_clk;
  logic i_rst_n;
  logic i_wr_req, i_rd_req;
  logic [AddrWidth-1:0] i_wr_addr, i_rd_addr;
  logic [DataWidth-1:0] i_wr_data, o_rd_data;

  sdram_ctrl #(
    .ClockFreq(ClockFreq)
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

  initial begin
    $dumpfile("sdram_ctrl_tb.vcd"); // Initialize VCD dump
    $dumpvars(0, sdram_ctrl_tb);    // Dump all variables in this module
    
    // Initial Signal Stimuli
    i_rst_n   <= '0; // Assert reset
    i_wr_req  <= '0;
    i_wr_data <= '0;
    i_wr_addr <= '0;
    i_rd_req  <= '0;
    i_rd_addr <= '0;
    repeat (2) @(posedge i_dram_clk);
    i_rst_n   <= '1;

    #(201000); // Wait for init to finish

    @(posedge i_dram_clk); // Sync back to clock
    i_wr_req  <= '1;
    i_wr_data <= $random;
    i_wr_addr <= {2'd0, 8'd5, 12'd13};
    @(posedge i_dram_clk);
    i_wr_req <= '0; // Deassert request
    @(posedge i_dram_clk);

    repeat (50) @(posedge i_dram_clk);

    @(posedge i_dram_clk); // Sync back to clock
    i_wr_req  <= '1;
    i_wr_data <= $random;
    i_wr_addr <= {2'd0, 8'd5, 12'd13};
    @(posedge i_dram_clk);
    i_wr_req <= '0; // Deassert request
    @(posedge i_dram_clk);

    repeat (50) @(posedge i_dram_clk);
    
    i_rd_req  <= '1;
    i_rd_addr <= {2'd0, 8'd5, 12'd13};
    @(posedge i_dram_clk);
    i_rd_req <= '0; // Deassert request
    @(posedge i_dram_clk);

    repeat(50) @(posedge i_dram_clk);

    i_rd_req  <= '1;
    i_rd_addr <= {2'd0, 8'd10, 12'd53};
    @(posedge i_dram_clk);
    i_rd_req <= '0; // Deassert request
    @(posedge i_dram_clk);

    repeat(50) @(posedge i_dram_clk);

    i_rd_req  <= '1;
    i_rd_addr <= {2'd0, 8'd15, 12'd53};
    @(posedge i_dram_clk);
    i_rd_req <= '0; // Deassert request
    @(posedge i_dram_clk);

    repeat(4000) @(posedge i_dram_clk);

    $display("sdram_ctrl testbench complete");
    $finish;
  end

endmodule
