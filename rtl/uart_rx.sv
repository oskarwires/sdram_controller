module uart_rx #(
  parameter  SystemClockFreq = 50_000_000,
  parameter  BaudRate        = 115200,
  parameter  Parity          = 1'b0,
  parameter  ParityEven      = 1'b0, // 1 if even, 0 if odd
  parameter  StopBit         = 1'b1,
  parameter  DataLength      = 8
)(
  /* Main Signals */
  input  logic                  i_clk, // Assuming this is at baudrate * oversampling
  input  logic 	                i_rst_n,
  /* UART Signals */
  input  logic                  i_rx,
  output logic [DataLength-1:0] o_rx_data,
  /* FIFO Signals */
  output logic                  o_rx_fifo_write_en
  /* Error Signals */
);
  
  localparam CyclesPerBit = SystemClockFreq / BaudRate;
  localparam ClkCounterWidth = $clog2(CyclesPerBit);
  localparam BitCounterWidth = $clog2(DataLength);
 
  typedef enum logic [2:0] {
    RESET  = 3'b000,
    WAIT   = 3'b001,
    START  = 3'b010,
    LOAD   = 3'b011,
    PARITY = 3'b100, 
    STOP   = 3'b101,
    READY  = 3'b110
  } states_t;

  states_t curr_state, next_state;

  logic [BitCounterWidth-1:0] bit_counter;
  logic [ClkCounterWidth-1:0] clk_counter;
  logic clk_counter_rst_n, bit_counter_rst_n, shift_reg_en;

  logic parity_bit;

  logic half_bit, strobe_bit;

  // State controller
  always_ff @(posedge i_clk, negedge i_rst_n) begin
    if (!i_rst_n) curr_state <= RESET;
    else          curr_state <= next_state;
  end

  // Next State Logic Controller
  always_comb begin
    unique case (curr_state)
      RESET:
        next_state   = WAIT;
      WAIT:
        if (i_rx == '0)
          next_state = START;
        else 
          next_state = WAIT;
      START:
        if (strobe_bit == '1)
	        next_state = LOAD;
	      else
	        next_state = START;
      LOAD:
        if (bit_counter == '0 && half_bit)
          case (Parity)
            0: next_state = STOP;
            1: next_state = PARITY;
          endcase
        else 
          next_state = LOAD;
      PARITY:
        if (half_bit) 
          next_state = STOP;
        else
          next_state = PARITY; 
      STOP:
        if (half_bit && i_rx) // halfway strobe AND rx == 1 (stop bit)
          next_state = READY;
        else 
          next_state = STOP;
      READY: 
        next_state   = WAIT;
    endcase
  end

  // FSM Output Controller
  always_comb begin
    unique case (curr_state)
      RESET:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b0000;
      WAIT:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b0000;
      START:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b1000;
      LOAD:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b1101;
      PARITY: {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b1000;
      STOP:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b1000;
      READY:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en} = 4'b0010;
    endcase
  end

  /* Incrementing Counter up to CyclesPerBit - 1 */
  always_ff @(posedge i_clk)
    if (!i_rst_n)
      clk_counter <= '0;
    else if (!clk_counter_rst_n)
      clk_counter <= '0;
    else if (clk_counter == CyclesPerBit - 1) // Reset counter if full
      clk_counter <= '0;
    else
      clk_counter <= clk_counter + 1'b1;

  /* Strobe and Half Bit Signal */
  always_comb begin
    if (clk_counter == CyclesPerBit / 2) begin
      strobe_bit = 1'b0;
      half_bit   = 1'b1;
    end else if (clk_counter == CyclesPerBit - 1) begin
      strobe_bit = 1'b1;
      half_bit   = 1'b0;
    end else begin
      strobe_bit = 1'b0;
      half_bit   = 1'b0;
    end
  end

  /* Shift Register */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      o_rx_data <= '0;
    else if (half_bit && shift_reg_en)
      o_rx_data <= {i_rx, o_rx_data[DataLength-1:1]};
  
  /* Decrementing Counter */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      bit_counter <= '1;
    else if (!bit_counter_rst_n)
      bit_counter <= '1;
    else if (half_bit)
      bit_counter <= bit_counter - 3'b1;  

endmodule

