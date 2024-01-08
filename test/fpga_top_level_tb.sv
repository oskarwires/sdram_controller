module fpga_top_level_tb();
  localparam ClockFreq = 133_000_000;
  localparam real ClockPeriod = 1_000_000_000.0 / ClockFreq;
  localparam BaudRate = 115200; // Baud rate
  localparam UartBitCycles = ClockFreq / BaudRate; // Clock cycles per UART bit

  real UartBitPeriodNs = 1_000_000_000.0 / BaudRate; // Bit period in nanoseconds
  real HalfBitPeriodNs = UartBitPeriodNs / 2.0;

  logic i_sys_clk, i_rst_n;
  logic i_rx, o_tx;
  logic [7:0] transmit_uart_packet;
  logic [7:0] recieve_uart_packet;

  initial i_sys_clk = 1'b0;
  always #(ClockPeriod / 2) i_sys_clk = ~i_sys_clk;

  fpga_top_level #(
    .ClockFreq(ClockFreq)
  ) uut (
    .i_sys_clk,   
    .o_sdram_clk(),  
    .i_rst_n,     
    .o_dram_addr(), 
    .o_dram_data(), 
    .o_dram_ba_0(), 
    .o_dram_ba_1(), 
    .o_dram_ldqm(), 
    .o_dram_udqm(), 
    .o_dram_we_n(), 
    .o_dram_cas_n(),
    .o_dram_ras_n(),
    .o_dram_cs_n(), 
    .o_dram_clk(),  
    .o_dram_cke(),
    .i_rx,
    .o_tx
  );

    // Task to transmit a UART packet to the UUT to test RX
  task automatic transmit_uart_stream (
    input logic [7:0] uart_packet
  );
    // Async assignment of value to model real world async conditions
    i_rx = 0; // Start Bit (0)
    #(UartBitPeriodNs); // Wait bit period

    // Transmitting DataLength data bits
    for (int i = 0; i < 8; i++) begin
      i_rx = uart_packet[i];      
      #(UartBitPeriodNs); // Wait bit period
    end

    i_rx = 1; // Stop bit (1)
    #(UartBitPeriodNs); // Wait bit period
  endtask

  task automatic recieve_uart_stream (
    output [7:0] uart_packet
  );
    @(negedge o_tx); // Wait for start bit (falling edge from 1 to 0)
      
    // Wait half a bit period to align to the middle of bits
    #(HalfBitPeriodNs);
    
    // Read each data bit
    for (int j = 0; j < 8; j++) begin
        #(UartBitPeriodNs);    // Wait bit period
        uart_packet[j] = o_tx; // Sample the data bit
    end
    #(UartBitPeriodNs); // Wait for stop bit

    assert(o_tx == 1'b1) else $error("No Stop bit detected"); // Stop bit requirement
  endtask

  initial begin
    $dumpfile("fpga_top_level_tb.vcd"); // Initialize VCD dump
    $dumpvars(0, fpga_top_level_tb);    // Dump all variables in this module
    i_rst_n   <= '0; // Assert reset
    i_rx      <= '1;
    repeat (2) @(posedge i_sys_clk);
    i_rst_n   <= '1;
    #(205000)
    
    repeat (10) @(posedge i_sys_clk);
    transmit_uart_packet = 8'b01010101;
    transmit_uart_stream(transmit_uart_packet);
    
    //recieve_uart_stream(recieve_uart_packet);

    $display("fpga top level test complete");
    $finish;
  end

endmodule
