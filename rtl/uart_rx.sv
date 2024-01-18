module uart_rx #(
  parameter  DataLength      = 8,
  parameter  SystemClockFreq = 50_000_000,
  parameter  BaudRate        = 115200,
  parameter  Parity          = 1'b0, // 1 if enabled, 0 if not
  parameter  ParityEven      = 1'b0  // 1 if even, 0 if odd
)(
  /* Main Signals */
  input  logic                  i_clk, // Assuming this is at baudrate * oversampling
  input  logic 	                i_rst_n,
  /* UART Signals */
  input  logic                  i_rx,
  output logic [DataLength-1:0] o_rx_data,
  /* FIFO Signals */
  output logic                  o_rx_fifo_write_en,
  input  logic                  i_rx_fifo_full,
  /* Error Signals */
  output logic                  o_rx_error /* Error, from invalid stop bit, needs clearing by rst_n assertion */
);
  
  localparam CyclesPerBit    = SystemClockFreq / BaudRate;
  localparam TenthOfBit      = CyclesPerBit / 10; // We will sample every half bit cycle, and a tenth before and after for majority three
  localparam ClkCounterWidth = $clog2(CyclesPerBit);
  localparam BitCounterWidth = $clog2(DataLength);
 
  typedef enum logic [2:0] {
    RESET,  // 000
    WAIT,   // 001
    START,  // 010
    LOAD,   // 011
    PARITY, // 100
    STOP,   // 101
    READY,  // 110
    ERROR   // 111
  } states_t;

  states_t curr_state, next_state;

  logic [BitCounterWidth-1:0] bit_counter;
  logic [ClkCounterWidth-1:0] clk_counter;
  logic clk_counter_rst_n, bit_counter_rst_n, shift_reg_en;

  logic parity_bit;

  logic half_bit, strobe_bit;

  logic [2:0] three_bits;  // The three bits sampled for majority three
  logic [1:0] sample_bits; // Which bit to sample for majority three
  // 0 = No sample
  // 1,2,3 = Sample bit 1, 2, or 3 for Majority Three

  function majority_three(input [2:0] bits);
    casez (bits)
      3'b11?:  majority_three = 1'b1;
      3'b1?1:  majority_three = 1'b1;
      3'b?11:  majority_three = 1'b1;
      default: majority_three = 1'b0;
    endcase
  endfunction

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
        if (!i_rx && !i_rx_fifo_full)
          next_state = START;
        else 
          next_state = WAIT;    // @ loopback
      START:
        if (strobe_bit && majority_three(three_bits))
          next_state = WAIT; // If the majority three isn't 0, then we must've had some noise. Cancel the reading
        else if (strobe_bit && !majority_three(three_bits)) // end of bit strobe AND rx == 0 (start bit)
	        next_state = LOAD;
	      else
	        next_state = START;   // @ loopback
      LOAD:
        if (bit_counter == '0 && strobe_bit)
          case (Parity)
            0: next_state = STOP;
            1: next_state = PARITY;
          endcase
        else 
          next_state = LOAD;    // @ loopback
      PARITY:
        if (half_bit) 
          next_state = STOP;
        else
          next_state = PARITY;  // @ loopback
      STOP:
        if (strobe_bit && majority_three(three_bits)) // end of bit strobe AND rx == 1 (stop bit)
          next_state = READY;
        else if (strobe_bit && !majority_three(three_bits)) // Invalid stop bit
          next_state = ERROR;
        else 
          next_state = STOP;    // @ loopback
      READY: 
        next_state   = WAIT;
      ERROR: // Reset must be asserted to get out of error state
        next_state   = ERROR;   // @ loopback
    endcase
  end

  // FSM Output Controller
  always_ff @(posedge i_clk) begin
    unique case (next_state)
      RESET:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b00000;
      WAIT:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b00000;
      START:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b10000;
      LOAD:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b11010;
      PARITY: {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b10000;
      STOP:   {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b10000;
      READY:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b00100;
      ERROR:  {clk_counter_rst_n, bit_counter_rst_n, o_rx_fifo_write_en, shift_reg_en, o_rx_error} <= 5'b00001;
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

  /* Strobe, Half bit, and Majority Three Sampling Signals */
  always_comb begin
    if (clk_counter == (CyclesPerBit / 2 - TenthOfBit)) begin
      strobe_bit   = 1'b0;
      sample_bits  = 2'd1; // Sample first bit for three
      half_bit     = 1'b0;
    end else if (clk_counter == CyclesPerBit / 2) begin
      strobe_bit   = 1'b0;
      sample_bits  = 2'd2; // Sample second bit for three
      half_bit     = 1'b1;
    end else if (clk_counter == (CyclesPerBit / 2 + TenthOfBit)) begin
      strobe_bit   = 1'b0;
      sample_bits  = 2'd3; // Sample the last bit for maj 3
      half_bit     = 1'b0;
    end else if (clk_counter == CyclesPerBit - 1) begin
      strobe_bit   = 1'b1;
      sample_bits  = '0;
      half_bit     = 1'b0;
    end else begin
      strobe_bit   = 1'b0;
      sample_bits  = '0;
      half_bit     = 1'b0;
    end
  end
  
  /* Majority Three Sampling */
  // Samples each bit three times, once at the middle of the bit, once 4/10th of the bit period, and once at 6/10th of the bit period
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      three_bits                <= '0;
    else if (sample_bits != '0)
      three_bits[sample_bits-1] <= i_rx; // I (cheekily) use the sample_bits from range 1..3, so that 0 is no sample, and 1 is first bit, etc.
    else
      three_bits                <= three_bits; // FF not latch!
  
  /* Shift Register */
  // Shift in the majority three voted value at the end of every bit
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      o_rx_data <= '0;
    else if (strobe_bit && shift_reg_en)
      o_rx_data <= {majority_three(three_bits), o_rx_data[DataLength-1:1]};
  
  /* Decrementing Counter */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      bit_counter <= DataLength - 1;
    else if (!bit_counter_rst_n)
      bit_counter <= DataLength - 1;
    else if (strobe_bit)
      bit_counter <= bit_counter - 3'b1;  
  
  start_bit:      assert property (@(posedge i_clk) (curr_state == WAIT && i_rx == 0) |-> ##1 (next_state == START)); // Do we transistion from 0?
  rx_fifo_full:   assert property (@(posedge i_clk) (i_rx_fifo_full) |-> !o_rx_fifo_write_en); // Don't write to full fifo!

  `ifdef FORMAL
    // SVA Assertions
    start_bit:    assert property (@(posedge i_clk) (curr_state == WAIT && i_rx == 0) |-> ##1 (next_state == START));
    rx_fifo_full: assert property (@(posedge i_clk) (i_rx_fifo_full) |-> !o_rx_fifo_write_en); // Don't write to full fifo!
  `endif

endmodule
