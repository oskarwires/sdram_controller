module uart_tx #(
  parameter SystemClockFreq = 50_000_000,
  parameter BaudRate        = 115200,
  parameter Parity          = 1'b0,
  parameter StopBit         = 1'b1,
  parameter DataLength      = 8,
  parameter FlowControl     = 1'b0
)(
  /* Main Signals */
  input  logic                  i_clk,
  input  logic 	                i_rst_n,
  /* UART Signals */
  output logic                  o_tx,
  input  logic                  i_cts,
  /* FIFO Signals */
  input  logic [DataLength-1:0] i_tx_fifo_data,
  input  logic                  i_tx_fifo_empty,
  output logic                  o_tx_fifo_read_en
);

  localparam CyclesPerBit    = SystemClockFreq / BaudRate;
  localparam ClkCounterWidth = $clog2(CyclesPerBit);
  localparam BitCounterWidth = $clog2(DataLength);

  logic [ClkCounterWidth-1:0] clk_counter;
  logic [BitCounterWidth-1:0] bit_counter;

  typedef enum logic [2:0] {
    RESET  = 3'b000,
    WAIT   = 3'b001,
    START  = 3'b010,
    DATA   = 3'b011,
    PARITY = 3'b100, 
    STOP   = 3'b101,
    DONE   = 3'b110
  } states_t;

  states_t curr_state, next_state;

  logic shift_reg_en, clk_counter_en, bit_counter_en;
  logic o_tx_d;

  // State Controller
  always_ff @(posedge i_clk, negedge i_rst_n) begin
    if (!i_rst_n) curr_state <= RESET;
    else          curr_state <= next_state;
  end

  // Next State Logic Controller
  always_comb begin
    unique case (curr_state)
      RESET:
        next_state = WAIT;
      WAIT:
        if (!i_tx_fifo_empty && (FlowControl ? i_cts : 1)) // Start transmitting if data in TX FIFO (and if FlowControl == 1, && i_cts)
          next_state = START;
        else 
          next_state = WAIT;
      START:
        if (clk_counter == (CyclesPerBit - 1))
          next_state = DATA;
        else 
          next_state = START;
      DATA:
        if (bit_counter == (DataLength - 1) && clk_counter == (CyclesPerBit - 1))
          case (Parity)
            0: next_state = STOP;
            1: next_state = PARITY;
          endcase
        else
          next_state = DATA;
      STOP:
        if (clk_counter == (CyclesPerBit - 1))
          next_state = DONE;
        else
          next_state = STOP;
      DONE:
        next_state = WAIT;
    endcase
  end 
  
  // FSM Output Controller
  always_comb begin
    unique case (curr_state)
      RESET:  {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b000, 1'b1};
      WAIT:   {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b000, 1'b1};
      START:  {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b100, 1'b0};
      DATA:   {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b110, i_tx_fifo_data[bit_counter]};
      PARITY: {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b100, 1'b1};
      STOP:   {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b100, 1'b1};
      DONE:   {clk_counter_en, bit_counter_en, o_tx_fifo_read_en, o_tx_d} = {3'b001, 1'b1}; // We read on the last stage as we utilize the FIFOs First-Word-Fall-Through, so this is just to load in the next data
    endcase
  end 

  /* TX Flip Flop, Sychronises the combinatorial output o_tx_d */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      o_tx <= '1; // Reset holds high
    else
      o_tx <= o_tx_d;

  /* Incrementing Clk Counter */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      clk_counter <= '0;
    else if (!clk_counter_en)
      clk_counter <= '0;
    else if (clk_counter == CyclesPerBit - 1)
      clk_counter <= '0;
    else
      clk_counter <= clk_counter + 1'b1;  

  /* Incrementing Bit Counter */
  always_ff @(posedge i_clk, negedge i_rst_n)
    if (!i_rst_n)
      bit_counter <= '0;
    else if (!bit_counter_en)
      bit_counter <= '0;
    else if (clk_counter == CyclesPerBit - 1)
      bit_counter <= bit_counter + 1'b1;  

endmodule
