`timescale 1ns / 1ps

module fifo_tb ();

  // Parameters
  localparam DataWidth = 8;
  localparam Depth     = 8;
  localparam PtrWidth  = $clog2(Depth);

  // Testbench Signals
  logic                 i_wr_clk = 1'b0;
  logic                 i_rd_clk = 1'b0;
  logic [DataWidth-1:0] i_wr_data;
  logic                 i_rst_n;
  logic                 i_wr_en;
  logic                 i_rd_en;
  logic [DataWidth-1:0] o_rd_data;
  logic                 o_full;
  logic                 o_empty;
  
  logic [DataWidth-1:0] test_inputs[8];

  // Instantiate the FIFO
  fifo #(
    .DataWidth(DataWidth),
    .Depth(Depth)
  ) uut (
    .i_wr_clk(i_wr_clk),
    .i_rd_clk(i_rd_clk),
    .i_wr_data(i_wr_data),
    .i_rst_n(i_rst_n),
    .i_wr_en(i_wr_en),
    .i_rd_en(i_rd_en),
    .o_rd_data(o_rd_data),
    .o_full(o_full),
    .o_empty(o_empty)
  );

  // Clock Generation
  initial   i_wr_clk = 1'b0;
  always #6 i_wr_clk = ~i_wr_clk;
  initial   i_rd_clk = 1'b1; // Assuming wr_clk starts with 0
  always #3.5 i_rd_clk = ~i_rd_clk;

  // Testbench Initial Block
  initial begin
    $dumpfile("fifo_tb.vcd"); // Initialize VCD dump
    $dumpvars(0, fifo_tb);    // Dump all variables in this module
    // Initialize signals
    i_wr_data <= 0;
    i_rst_n <= 0;
    i_wr_en <= 0;
    i_rd_en <= 0;
      
    #10; // Wait for a couple of clock cycles
    i_rst_n <= 1;
    #20;
    i_rst_n <= 0;
    #10;
    i_rst_n <= 1;
    #10;
    
    // TEST 1:
    // Randomize the input vector
    for (int i = 0; i < 8; i++) begin
      test_inputs[i] <= $random; // Masking to get the lower 8 bits
    end
    @(posedge i_rd_clk);
    
    // Write to FIFO
    @(posedge i_wr_clk);
    for (int i = 0; i < Depth; i++) begin
      i_wr_data <= test_inputs[i]; // Assign data
      i_wr_en <= 1; // We want to write data!
      @(posedge i_wr_clk); // Wait for next rising edge
    end
    i_wr_data <= $random; // Assign random value to in_data, which should NOT be written
    i_wr_en <= 0;
    @(posedge i_wr_clk);
    assert(o_full == 1'b1) else $error("Full not asserted");
    wait(!o_empty);
     
    // Read from FIFO
    for (int i = 0; i < Depth; i++) begin
      i_rd_en <= 1;
      @(posedge i_rd_clk); 
      assert(o_rd_data == test_inputs[i]) else $error("Outputted data %h does not equal input value %h", o_rd_data, test_inputs[i]);
    end
    i_rd_en <= 0;

    // Test 2: Simultaneous read and write
    // Randomize the input vector
    for (int i = 0; i < 8; i++) begin
      test_inputs[i] <= $random; // Masking to get the lower 8 bits
    end
    @(posedge i_rd_clk);
    
    fork
       // Write to FIFO
       begin
        @(posedge i_wr_clk);
        for (int i = 0; i < Depth; i++) begin
          i_wr_data <= test_inputs[i]; // Assign data
          i_wr_en <= 1; // We want to write data!
          @(posedge i_wr_clk); // Wait for next rising edge
        end
        i_wr_data <= $random; // Assign random value to in_data, which should NOT     be written
        i_wr_en <= 0;
       end

       begin
        wait(!o_empty);
        // Read from FIFO
        for (int i = 0; i < Depth; i++) begin
          i_rd_en <= 1;
          @(posedge i_rd_clk); 
          assert(o_rd_data == test_inputs[i]) else $error("Outputted data %h does     not equal input value %h", o_rd_data, test_inputs[i]);
        end
        i_rd_en <= 0;
       end
    join
  
    // Additional test cases...
    #100;
    $display("FIFO Test Complete");
    $finish;
  end
endmodule
