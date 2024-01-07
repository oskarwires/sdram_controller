`timescale 1ns / 1ps
module sdram_ctrl #(
  parameter  AddrWidth      = 13,
  parameter  DataWidth      = 16,
  parameter  CasLatency     = 3,
  parameter  ClockFreq      = 100_000_000,  /* MHz of DRAM Clk */
  parameter  WaitTime       = 200,         /* Microseconds */
  parameter  TrpTime        = 20,          /* Nanoseconds */
  parameter  TarfcTime      = 70,          /* Nanoseconds */
  localparam CyclesPerWait  = ClockFreq / (1_000_000 / WaitTime),
  localparam CyclesPerTrp   = ClockFreq / (1_000_000_000 / TrpTime),
  localparam CyclesPerTarfc = ClockFreq / (1_000_000_000 / TarfcTime),
  localparam CounterWidth   = $clog2(ClockFreq)
)(
  /* System Signals */
  input  logic                 i_sys_clk,    /* System Clock Frequency */
  input  logic                 i_dram_clk,   /* PLL Generated DRAM Clock */
  input  logic                 i_rst_n,      /* Sync Active Low Reset */
  /* ----- User signals ----- */
  /* Write Port */
  input  logic                 i_wr_req,
  input  logic [AddrWidth-1:0] i_wr_addr,
  input  logic [DataWidth-1:0] i_wr_data,
  /* Read Port */
  input  logic                 i_rd_req,
  input  logic [AddrWidth-1:0] i_rd_addr,
  output logic [DataWidth-1:0] o_rd_data,
  /* ----- SDRAM signals ----- */
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
  output logic                 o_dram_cke    /* Clock Enable */
);

  /* dram_cmd_t = {CS, RAS, CAS, WE} */
  typedef enum logic [3:0] {
    CMD_NOP      = 4'b0111,
    CMD_RD_RDA   = 4'b0101,
    CMD_WR_WRA   = 4'b0100,
    CMD_ACT      = 4'b0011,
    CMD_PRE_PALL = 4'b0010,
    CMD_MRS      = 4'b0000,
    CMD_REF      = 4'b0001
  } dram_cmd_t;
  
  typedef enum logic [3:0] {
    INIT_RESET,
    INIT_WAIT,
    INIT_PALL,
    INIT_WAIT_TRP,
    INIT_REF_1,
    INIT_WAIT_TARFC_1,
    INIT_REF_2,
    INIT_WAIT_TARFC_2,
    INIT_MRS,
    INIT_WAIT_TMRD,
    CMD_STATE_NOP
    // ...
  } dram_states_t;

  dram_states_t curr_state, next_state;

  logic [CounterWidth-1:0] counter;

  logic counter_rst_n;

  logic a10;
  
  logic [3:0] cmd;

  assign o_dram_cke      = 1'b1;
  assign o_dram_addr[10] = a10;
  assign {o_dram_cs_n, o_dram_ras_n, o_dram_cas_n, o_dram_we_n} = cmd;

  // State controller
  always_ff @(posedge i_dram_clk, negedge i_rst_n) begin
    if (!i_rst_n) curr_state <= INIT_RESET;
    else          curr_state <= next_state;
  end

  // Next State Logic Controller
  always_comb begin
    unique case (curr_state)
      INIT_RESET: begin
        next_state = INIT_WAIT;
      end
      INIT_WAIT: begin
        if (counter == CyclesPerWait - 1) begin
          next_state = INIT_PALL;
        end else begin
          next_state = INIT_WAIT;
        end
      end
      INIT_PALL: begin
        next_state = INIT_WAIT_TRP;
      end
      INIT_WAIT_TRP: begin
        if (counter == CyclesPerTrp - 1) begin
          next_state = INIT_REF_1;
        end else begin
          next_state = INIT_WAIT_TRP;
        end
      end
      INIT_REF_1: begin
        next_state = INIT_WAIT_TARFC_1;
      end
      INIT_WAIT_TARFC_1: begin
        if (counter == CyclesPerTarfc - 1) begin
          next_state = INIT_REF_2;
        end else begin
          next_state = INIT_WAIT_TARFC_1;
        end
      end
      INIT_REF_2: begin
        next_state = INIT_WAIT_TARFC_2;
      end
      INIT_WAIT_TARFC_2: begin
        if (counter == CyclesPerTarfc - 1) begin
          next_state = INIT_MRS;
        end else begin
          next_state = INIT_WAIT_TARFC_2;
        end
      end
      INIT_MRS: begin
        next_state = INIT_MRS;
      end
    endcase
  end

  // FSM Output Controller
  always_comb begin
    unique case (curr_state)
      INIT_RESET:        {counter_rst_n, cmd, a10} = {1'b0, CMD_NOP, 1'b0};
      INIT_WAIT:         {counter_rst_n, cmd, a10} = {1'b1, CMD_NOP, 1'b0};
      INIT_PALL:         {counter_rst_n, cmd, a10} = {1'b0, CMD_PRE_PALL, 1'b1};
      INIT_WAIT_TRP:     {counter_rst_n, cmd, a10} = {1'b1, CMD_NOP, 1'b0};
      INIT_REF_1:        {counter_rst_n, cmd, a10} = {1'b0, CMD_REF, 1'b0};
      INIT_WAIT_TARFC_1: {counter_rst_n, cmd, a10} = {1'b1, CMD_NOP, 1'b0};
      INIT_REF_2:        {counter_rst_n, cmd, a10} = {1'b0, CMD_REF, 1'b0};
      INIT_WAIT_TARFC_2: {counter_rst_n, cmd, a10} = {1'b1, CMD_NOP, 1'b0};
      INIT_MRS:          {counter_rst_n, cmd, a10} = {1'b1, CMD_MRS, 1'b0}; /* Here Mode Reg is set based on o_dram_addr */
    endcase
  end

  /* Incrementing Counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      counter <= '0;
    else if (!counter_rst_n)
      counter <= '0;
    else
      counter <= counter + 1'b1;  

endmodule
