module fpga_top_level #(
  parameter ClockFreq = 133_000_000,
  parameter AddrWidth = 22,
  parameter DataWidth = 16,
  parameter SendDelay = 100 /* Clock Cycles */
)(
  /* ----- Main Signals ----- */
  input  logic                 i_sys_clk,
  output logic                 o_sdram_clk, /* A copy for signal analyzer */
  input  logic                 i_rst_n,
  /* ----- SDRAM Signals ----- */
  output logic [AddrWidth-1:0] o_dram_addr,  /* Read/Write Address */
  inout  tri   [DataWidth-1:0] o_dram_data,  /* Read/Write Data */
  output logic                 o_dram_ba_0,  /* Bank Address [0] */
  output logic                 o_dram_ba_1,  /* Bank Address [1] */
  output logic                 o_dram_ldqm,  /* Low byte data mask */
  output logic                 o_dram_udqm,  /* High byte data mask */
  output logic                 o_dram_we_n,  /* Write enable */
  output logic                 o_dram_cas_n, /* Column address strobe */
  output logic                 o_dram_ras_n, /* Row address strobe */
  output logic                 o_dram_cs_n,  /* Chip select */
  output logic                 o_dram_clk,   /* DRAM Clock */
  output logic                 o_dram_cke,   /* Clock Enable */
  /* ----- UART Signals ----- */
  input  logic                 i_rx,
  output logic                 o_tx
); 

  localparam CounterWidth = $clog2(SendDelay);

  logic [CounterWidth-1:0] send_counter;
  logic counter_en;

  logic [7:0] uart_packet;
  logic [7:0] uart_tx_data, uart_rx_data;
  logic uart_tx_req, uart_tx_rdy, uart_rx_req, uart_rx_rdy;
  
  logic dram_wr_req, dram_rd_req, dram_rd_rdy;
  logic [DataWidth-1:0] dram_wr_data, dram_rd_data;
  logic [AddrWidth-1:0] dram_wr_addr, dram_rd_addr;

  logic write_dram_en;
  logic read_dram_en;
 
  uart #(
	  .BaudRate(115200),
	  .SystemClockFreq(ClockFreq)
  ) uart (
    .i_rst_n,
	  .i_clk(o_sdram_clk),
	  .i_rx,
	  .o_tx,
    .i_tx_data(uart_tx_data),
    .o_rx_data(uart_rx_data),
    .o_rx_rdy(uart_rx_rdy),
    .o_tx_rdy(uart_tx_rdy),
    .i_rx_req(uart_rx_req),
    .i_tx_req(uart_tx_req)
  );

  sdram_ctrl #(
    .ClockFreq(ClockFreq)
  ) sdram_ctrl (
    .i_sys_clk(o_sdram_clk),
    .i_dram_clk(o_sdram_clk),
    .i_rst_n,
    .i_wr_req(dram_wr_req),
    .i_wr_addr(dram_wr_addr),
    .i_wr_data(dram_wr_data),
    .i_rd_req(dram_rd_req),
    .i_rd_addr(dram_rd_addr),
    .o_rd_data(dram_rd_data),
    .o_rd_rdy(dram_rd_rdy),
    .o_dram_addr, 
    .o_dram_data, 
    .o_dram_ba_0, 
    .o_dram_ba_1, 
    .o_dram_ldqm, 
    .o_dram_udqm, 
    .o_dram_we_n, 
    .o_dram_cas_n,
    .o_dram_ras_n,
    .o_dram_cs_n, 
    .o_dram_clk,  
    .o_dram_cke   
  );

  /* PLL GOES HERE */
  assign o_sdram_clk = i_sys_clk; // CLOCK

  always_ff @(posedge o_sdram_clk) begin
    if (uart_rx_rdy && !uart_rx_req) begin
      uart_rx_req   <= '1;
      uart_packet   <= uart_rx_data;
      write_dram_en <= '1;
    end else begin
      uart_rx_req   <= '0;
      write_dram_en <= '0;
    end
  end

  always_ff @(posedge o_sdram_clk) begin
    if (dram_rd_rdy && uart_tx_rdy) begin
      uart_tx_req  <= '1;
      uart_tx_data <= dram_rd_data;
    end else begin
      uart_tx_req  <= '0;
      uart_tx_data <= '0;
    end

    if (write_dram_en) begin
      dram_wr_req  <= '1;
      dram_wr_addr <= '0;
      dram_wr_data <= uart_packet;
      counter_en   <= '1;
    end else begin
      dram_wr_req  <= '0;
      dram_wr_addr <= '0;
      dram_wr_data <= uart_packet;
      counter_en   <= '0;
    end

    if (read_dram_en) begin
      dram_rd_req  <= '1;
      dram_rd_addr <= '0;
    end else begin
      dram_rd_req  <= '0;
      dram_rd_addr <= '0;
    end
  end

  /* Incrementing Counter */
  always_ff @(posedge o_sdram_clk) begin
    if (!i_rst_n)
      send_counter <= '0;
    if (send_counter == SendDelay - 1)
      send_counter <= '0;
    if (counter_en || send_counter != '0) 
      send_counter <= send_counter + 1'b1;
  end

  /* Timer to read SDRAM and Send value back over UART */
  always_comb 
    if (send_counter == SendDelay - 1)
      read_dram_en = '1;
    else 
      read_dram_en = '0;

endmodule
