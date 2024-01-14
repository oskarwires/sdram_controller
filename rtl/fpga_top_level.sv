module fpga_top_level #(
  parameter ClockFreq = 133_000_000,
  parameter IAddrWidth = 22,
  parameter OAddrWidth = 12,
  parameter DataWidth = 16,
  parameter SendDelay = 100 /* Clock Cycles */
)(
  /* ----- Main Signals ----- */
  input  logic                  i_sys_clk,
  output logic                  o_sdram_clk,    /* A copy for signal analyzer */
  input  logic                  i_rst_n,
  /* ----- SDRAM Signals ----- */
  output logic [OAddrWidth-1:0] o_dram_addr,    /* Read/Write Address */
  inout  tri   [DataWidth-1:0]  io_dram_data,   /* Read/Write Data */
  output logic                  o_dram_ba_0,    /* Bank Address [0] */
  output logic                  o_dram_ba_1,    /* Bank Address [1] */
  output logic                  o_dram_ldqm,    /* Low byte data mask */
  output logic                  o_dram_udqm,    /* High byte data mask */
  output logic                  o_dram_we_n,    /* Write enable */
  output logic                  o_dram_cas_n,   /* Column address strobe */
  output logic                  o_dram_ras_n,   /* Row address strobe */
  output logic                  o_dram_cs_n,    /* Chip select */
  output logic                  o_dram_clk,     /* DRAM Clock */
  output logic                  o_dram_cke,     /* Clock Enable */
  /* ----- UART Signals ----- */
  input  logic                  i_rx,
  output logic                  o_tx
); 

  localparam CounterWidth = $clog2(SendDelay);

  logic [CounterWidth-1:0] send_counter;
  logic counter_en;
  
  logic [7:0] uart_packet;
  logic [7:0] uart_tx_data, uart_rx_data;
  logic uart_tx_req, uart_tx_rdy, uart_rx_req, uart_rx_rdy;
  
  logic dram_wr_req, dram_rd_req, dram_rd_rdy;
  logic [DataWidth-1:0] dram_wr_data, dram_rd_data;
  logic [IAddrWidth-1:0] dram_wr_addr, dram_rd_addr;

  logic write_dram_en;
  logic read_dram_en;
  logic dram_clk;

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
    .i_tx_req(uart_tx_req),
    .i_cts(),
    .o_rts(),
    .i_ctrl(),
    .o_status()
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
    .io_dram_data, 
    .o_dram_ba_0, 
    .o_dram_ba_1, 
    .o_dram_ldqm, 
    .o_dram_udqm, 
    .o_dram_we_n, 
    .o_dram_cas_n,
    .o_dram_ras_n,
    .o_dram_cs_n, 
    .o_dram_clk(dram_clk),  
    .o_dram_cke   
  );

  /* PLL GOES HERE */
  /*
  pll pll (
    .inclk0(i_sys_clk),
	 .c0(o_sdram_clk)
  );
  */
  assign o_dram_clk = ~dram_clk;
  assign o_sdram_clk = i_sys_clk;

  typedef enum logic [3:0] {
    RESET,
    WAITING,
    UART_RECEIVE,
    UART_DECODE,
    UART_INVALID,
    WRITE_ADDR,
    WRITE_WAIT,
    WRITE_DATA,
    WRITE_CMD,
    READ_ADDR,
    READ_CMD,
    UART_TRANSMIT
  } ctrl_state_t;

  ctrl_state_t next_state, curr_state;

  // State controller
  always_ff @(posedge o_sdram_clk) begin
    if (!i_rst_n) curr_state <= RESET;
    else          curr_state <= next_state;
  end

  // Next State Logic Controller
  always_comb begin
    unique case (curr_state)

      RESET:
        next_state = WAITING;

      WAITING:
        if (uart_rx_rdy)
          next_state = UART_RECEIVE;
        else
          next_state = WAITING;

      UART_RECEIVE:
        next_state = UART_DECODE;

      UART_DECODE:
        if (uart_packet == 8'h77) begin // ASCII "w" for write
          // Write at next given address, with 8 bit sign extended data after
          next_state = WRITE_ADDR;
        end else if (uart_packet == 8'h72) begin // ASCII "r" for read
          // Read at next given address
          next_state = READ_ADDR;
        end else begin
          next_state = UART_INVALID;
        end

      UART_INVALID:
        next_state = WAITING;

      READ_ADDR:
        if (uart_rx_rdy)
          next_state = READ_CMD;
        else
          next_state = READ_ADDR;

      READ_CMD:
        next_state = UART_TRANSMIT;

      UART_TRANSMIT:
        if (dram_rd_rdy) // Wait for DRAM Controller to have read data ready before we send it
          next_state = WAITING; 
        else
          next_state = UART_TRANSMIT;

      WRITE_ADDR:
        if (uart_rx_rdy)
          next_state = WRITE_WAIT;
        else 
          next_state = WRITE_ADDR;

      WRITE_WAIT:
        next_state = WRITE_DATA;

      WRITE_DATA:
        if (uart_rx_rdy)
          next_state = WRITE_CMD;
        else 
          next_state = WRITE_DATA;

      WRITE_CMD:
        next_state = WAITING;
    endcase
  end

  // FSM Output Controller
  always_comb begin
    case (curr_state) 
      READ_CMD: begin
        dram_rd_req  <= '1;
        dram_wr_req  <= '0;
      end
      WRITE_CMD: begin
        dram_rd_req  <= '0;
        dram_wr_req  <= '1;
      end
      default: begin
        dram_rd_req  <= '0;
        dram_wr_req  <= '0;
      end
    endcase
  end

  // TRANSMIT RECIEVED SDRAM DATA
  always_ff @(posedge o_sdram_clk) begin
    if (curr_state == UART_TRANSMIT && dram_rd_rdy && uart_tx_rdy) begin
      uart_tx_data <= dram_rd_data[7:0];
      uart_tx_req  <= '1;
    end else begin
      uart_tx_data <= '0;
      uart_tx_req  <= '0;
    end
  end
  
  // SAMPLE UART DATA
  always_ff @(posedge o_sdram_clk) begin
    if (uart_rx_rdy && !uart_rx_req) begin
      if (curr_state == WRITE_ADDR) begin
        dram_wr_addr    <= {14'b0, uart_rx_data};
        uart_rx_req     <= 1'b1;
      end if (curr_state == UART_RECEIVE) begin
        uart_packet     <= uart_rx_data;
        uart_rx_req     <= 1'b1;
      end if (curr_state == WRITE_DATA) begin
        dram_wr_data    <= {8'b0, uart_rx_data};
        uart_rx_req     <= 1'b1;
      end if (curr_state == READ_ADDR) begin
        dram_rd_addr    <= {14'b0, uart_rx_data};
        uart_rx_req     <= 1'b1;
      end
    end else begin
      uart_rx_req     <= 1'b0;
    end
  end

endmodule
