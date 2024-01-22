// The purpose of this module is to allow my FPGA to act as a testbench, with UART being used to issue read and write commands to the FPGA
module fpga_top_level #(
  parameter ClockFreq   = 100_000_000,
  parameter IAddrWidth  = 22,
  parameter OAddrWidth  = 12,
  parameter DataWidth   = 16,
  parameter BurstLength = 4
)(
  /* ----- Main Signals ----- */
  input  logic                  i_sys_clk,
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

  localparam WordCounterWidth = $clog2(BurstLength);

  logic counter_en;
  
  logic [7:0] uart_packet;
  logic [7:0] uart_tx_data, uart_rx_data;
  logic uart_tx_req, uart_tx_rdy, uart_rx_req, uart_rx_rdy;
  
  logic dram_wr_req, dram_rd_req, dram_rd_rdy;
  logic [DataWidth-1:0]  dram_wr_data [BurstLength];
  logic [DataWidth-1:0]  dram_rd_data [BurstLength];
  logic [IAddrWidth-1:0] dram_wr_addr, dram_rd_addr;

  logic write_dram_en;
  logic read_dram_en;
  logic dram_clk;

  uart #(
	  .BaudRate(115200),
	  .SystemClockFreq(ClockFreq)
  ) uart (
    .i_rst_n,
	  .i_clk(dram_clk),
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
    .o_rx_error()
  );

  sdram_ctrl #(
    .ClockFreq(ClockFreq),
    .BurstLength(BurstLength)
  ) sdram_ctrl (
    .i_sys_clk(dram_clk),
    .i_dram_clk(dram_clk),
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
    .o_dram_clk(),  
    .o_dram_cke   
  );

  /* PLL GOES HERE */
  /*
  pll pll (
    .inclk0(i_sys_clk),
	  .c0(dram_clk),  // SDRAM Clock for controller
	  .c1(o_dram_clk) // Same frequency as above, but phase inverted by 180 degrees for output
  );
  */

  logic [WordCounterWidth-1:0] word_counter; 

  assign o_dram_clk = ~i_sys_clk; // For the TB, our system input clock is 100 MHz
  assign dram_clk   = i_sys_clk;

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
  always_ff @(posedge dram_clk)
    if (!i_rst_n) curr_state <= RESET;
    else          curr_state <= next_state;

  // Next State Logic Controller
  always_comb begin
    unique case (curr_state)
  
      RESET:                                                             next_state = WAITING;
                                          
      WAITING: if (uart_rx_rdy)                                          next_state = UART_RECEIVE;
               else                                                      next_state = WAITING; // @ loopback
                                          
      UART_RECEIVE:                                                      next_state = UART_DECODE;
    
      UART_DECODE: if (uart_packet == 8'h77) begin             // ASCII "w" for write
                                                                         next_state = WRITE_ADDR; // Write at next given address, with 8 bit zero extended data after
                   end else if (uart_packet == 8'h72) begin    // ASCII "r" for read
                                                                         next_state = READ_ADDR;  // Read at next given address
                   end else                                              next_state = UART_INVALID;  
                                         
      UART_INVALID:                                                      next_state = WAITING;
                                       
      READ_ADDR: if (uart_rx_rdy)                                        next_state = READ_CMD;
                 else                                                    next_state = READ_ADDR; // @ loopback
                                       
      READ_CMD:                                                          next_state = UART_TRANSMIT;

      UART_TRANSMIT: if (BurstLength == 1) 
                       if (dram_rd_rdy)                                  next_state = WAITING;             
                       else                                              next_state = UART_TRANSMIT; // @ loopback
                     else 
                       if (word_counter == BurstLength - 1)              next_state = WAITING; 
                       else                                              next_state = UART_TRANSMIT; // @ loopback
    
      WRITE_ADDR: if (uart_rx_rdy)                                       next_state = WRITE_WAIT;
                  else                                                   next_state = WRITE_ADDR; // @ loopback
                                  
      WRITE_WAIT:                                                        next_state = WRITE_DATA;
                                  
      WRITE_DATA: if (uart_rx_rdy)                                       next_state = WRITE_CMD;
                  else                                                   next_state = WRITE_DATA; // @ loopback
                                  
      WRITE_CMD:                                                         next_state = WAITING;
    endcase
  end

  // FSM Output Controller, not registered
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
  always_ff @(posedge dram_clk) begin
    if ((curr_state == UART_TRANSMIT && dram_rd_rdy && uart_tx_rdy && word_counter == '0) || (curr_state == UART_TRANSMIT && word_counter != BurstLength && word_counter != '0)) begin
      uart_tx_data <= dram_rd_data[word_counter][7:0];
      uart_tx_req  <= '1;
      word_counter <= word_counter + 1'b1;
    end else begin
      uart_tx_data <= '0;
      uart_tx_req  <= '0;
      word_counter <= '0;
    end
  end
  
  // SAMPLE UART DATA
  always_ff @(posedge dram_clk) begin
    if (uart_rx_rdy && !uart_rx_req) begin
      if (curr_state == WRITE_ADDR) begin
        dram_wr_addr      <= {14'b0, uart_rx_data};
        uart_rx_req       <= 1'b1;
      end if (curr_state == UART_RECEIVE) begin
        uart_packet       <= uart_rx_data;
        uart_rx_req       <= 1'b1;
      end if (curr_state == WRITE_DATA) begin
        for (int i = 0; i < BurstLength; i++) begin
          dram_wr_data[i] <= {8'b0, uart_rx_data+i};
        end
        uart_rx_req       <= 1'b1;
      end if (curr_state == READ_ADDR) begin
        dram_rd_addr      <= {14'b0, uart_rx_data};
        uart_rx_req       <= 1'b1;
      end
    end else begin
      uart_rx_req         <= 1'b0;
    end
  end

endmodule
