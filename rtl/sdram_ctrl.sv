`timescale 1ns / 1ps
module sdram_ctrl #(
  parameter  RowWidth       = 12,
  parameter  ColWidth       = 8,
  parameter  BankWidth      = 2,
  parameter  DataWidth      = 16,
  parameter  CasLatency     = 3,
  parameter  ClockFreq      = 100_000_000,  /* MHz of DRAM Clk */
  parameter  WaitTime       = 200,          /* Microseconds */
  parameter  TrpTime        = 20,           /* Nanoseconds */
  parameter  TrcdTime       = 20,           /* Nanoseconds */
  parameter  TarfcTime      = 70,           /* Nanoseconds */
  localparam AddrWidth      = BankWidth + ColWidth + RowWidth,
  localparam CyclesPerWait  = ClockFreq / (1_000_000 / WaitTime),
  localparam CyclesPerTrp   = ClockFreq / (1_000_000_000 / TrpTime),
  localparam CyclesPerTarfc = ClockFreq / (1_000_000_000 / TarfcTime),
  localparam CyclesPerTrcd  = ClockFreq / (1_000_000_000 / TrcdTime),
  localparam CyclesPerTmrd  = 2,            /* 2 Clock Cycles */
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
 
  logic refresh_en, refresh_req;

  sdram_refresh #(
    .ClockFreq(ClockFreq)
  ) sdram_refresh (
    .i_dram_clk,
    .i_rst_n,
    .i_refresh_en(refresh_en),
    .o_refresh_req(refresh_req)
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
  
  typedef enum logic [4:0] { /* Decrease this bit width to whatever we need at the end */
    INIT_RESET = 5'b00000,
    INIT_WAIT = 5'b00001,
    INIT_PALL = 5'b00010,
    INIT_WAIT_TRP = 5'b00011,
    INIT_REF_1 = 5'b00100,
    INIT_WAIT_TARFC_1 = 5'b00101,
    INIT_REF_2 = 5'b00110,
    INIT_WAIT_TARFC_2 = 5'b00111,
    INIT_MRS = 5'b01000,
    INIT_WAIT_TMRD = 5'b01001,
    RDY_NOP = 5'b01010,
    EXEC_REF = 5'b01011,
    EXEC_WRITE_ACT,
    EXEC_WRITE_WAIT_TRCD,
    EXEC_WRITE_WRITE,
    EXEC_READ_ACT
    // ...
  } dram_states_t;

  dram_states_t curr_state, next_state;

  logic [CounterWidth-1:0] counter;

  logic counter_rst_n;
  
  logic [3:0] cmd;
  logic write_enable;
  logic [DataWidth-1:0] dram_data_out;

  assign o_dram_cke      = 1'b1;
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
        next_state = INIT_WAIT_TMRD;
      end
      INIT_WAIT_TMRD: begin
        if (counter == CyclesPerTmrd - 1) begin
          next_state = RDY_NOP;
        end else begin
          next_state = INIT_WAIT_TMRD;
        end
      end
      RDY_NOP: begin
        if (refresh_req) begin
          next_state = EXEC_REF;
        end else if (i_wr_req) begin
          next_state = EXEC_WRITE_ACT;
        end else if (i_rd_req) begin
          next_state = EXEC_READ_ACT;
        end else begin
          next_state = RDY_NOP;
        end
      end
      EXEC_REF: begin
        next_state = RDY_NOP;
      end
      EXEC_WRITE_ACT: begin
        next_state = EXEC_WRITE_WAIT_TRCD;
      end
      EXEC_WRITE_WAIT_TRCD: begin
        if (counter == CyclesPerTrcd - 1) begin
          next_state = EXEC_WRITE_WRITE;
        end else begin
          next_state = EXEC_WRITE_WAIT_TRCD;
        end
      end
      EXEC_WRITE_WRITE: begin
        next_state = RDY_NOP;
      end
    endcase
  end

  // FSM Output Controller
  always_comb begin
    unique case (curr_state)

      INIT_RESET: begin
        counter_rst_n = 1'b0;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_WAIT: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_PALL: begin
        counter_rst_n = 1'b0;
        cmd = CMD_PRE_PALL;
        refresh_en = 1'b0;
        o_dram_addr = {1'b0, 1'b1, 10'b0};
        write_enable = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_WAIT_TRP: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_REF_1: begin
        counter_rst_n = 1'b0;
        cmd = CMD_REF;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_WAIT_TARFC_1: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_REF_2: begin   
        counter_rst_n = 1'b0;
        cmd = CMD_REF;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_WAIT_TARFC_2: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_MRS: begin /* Here Mode Reg is set based on o_dram_addr */
        counter_rst_n = 1'b0;
        cmd = CMD_MRS;
        /* {A[11] = 0, A[10] = 0, A[9] = WB, A[8] = 0, A[7] = 0, A[6:4] = CAS Latency, A[3] = Burst Type, A[2:0] = Burst Length} */
        o_dram_addr = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 3'b011, 1'b0, 3'b000}; 
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      INIT_WAIT_TMRD: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b0;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      RDY_NOP: begin
        counter_rst_n = 1'b0;
        cmd = CMD_NOP;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b1;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      EXEC_REF: begin
        counter_rst_n = 1'b0;
        cmd = CMD_REF;
        o_dram_addr = '0;
        write_enable = 1'b0;
        refresh_en = 1'b1;
        {o_dram_ba_0, o_dram_ba_1} = 2'b00;
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      EXEC_WRITE_ACT: begin
        counter_rst_n = 1'b0;
        cmd = CMD_ACT;
        o_dram_addr = i_wr_addr; /* {A[0:11] = Rows} */
        write_enable = 1'b0;
        refresh_en = 1'b1;
        {o_dram_ba_0, o_dram_ba_1} = i_wr_addr[BankWidth-1:0];
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      EXEC_WRITE_WAIT_TRCD: begin
        counter_rst_n = 1'b1;
        cmd = CMD_NOP;
        o_dram_addr = i_wr_addr[BankWidth+ColWidth+RowWidth-1:BankWidth+ColWidth];
        write_enable = 1'b0;
        refresh_en = 1'b1;
        {o_dram_ba_0, o_dram_ba_1} = i_wr_addr[BankWidth-1:0];
        {o_dram_ldqm, o_dram_udqm} = 2'b11;
      end

      EXEC_WRITE_WRITE: begin
        counter_rst_n = 1'b0;
        cmd = CMD_WR_WRA;
        o_dram_addr = {1'b0, 1'b1, 1'b1, 0'b0, i_wr_addr[BankWidth+ColWidth-1:BankWidth]}; /* {A[11] = ?, A[10] = Auto Precharge, A[9] = Single Write, A[8] = ?,  A[0:7] = Cols} */
        write_enable = 1'b1;
        refresh_en = 1'b1;
        {o_dram_ba_0, o_dram_ba_1} = i_wr_addr[BankWidth-1:0];
        {o_dram_ldqm, o_dram_udqm} = 2'b00; /* Low so we can control the data buffer, DQM Write Latency is 0 cycles */
      end

    endcase
  end

  always_comb begin
    if (write_enable) begin
        dram_data_out = i_wr_data;  // Drive the data line during write
    end else begin
        dram_data_out = {DataWidth{1'bz}};  // High-impedance state during read
    end
  end
  assign o_dram_data = dram_data_out;

  /* Incrementing Counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      counter <= '0;
    else if (!counter_rst_n)
      counter <= '0;
    else
      counter <= counter + 1'b1;  

endmodule
