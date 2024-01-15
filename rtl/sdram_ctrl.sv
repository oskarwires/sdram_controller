`timescale 1ns / 1ps
module sdram_ctrl #(
  parameter  RowWidth       = 12,
  parameter  ColWidth       = 8,
  parameter  BankWidth      = 2,
  parameter  DataWidth      = 16,
  parameter  CasLatency     = 3,
  parameter  ClockFreq      = 133_000_000,  /* MHz of DRAM Clk */
  parameter  WaitTime       = 200,          /* Microseconds */
  parameter  TrpTime        = 20,           /* Nanoseconds */
  parameter  TrcdTime       = 20,           /* Nanoseconds */
  parameter  TarfcTime      = 70,           /* Nanoseconds */
  parameter  OAddrWidth     = 12,
  /* {BA1, BA0, Col[7:0], Row[11:0]} */
  localparam IAddrWidth     = BankWidth + ColWidth + RowWidth, 
  localparam CyclesPerWait  = ClockFreq / (1_000_000 / WaitTime),
  localparam CyclesPerTrp   = ClockFreq / (1_000_000_000 / TrpTime) + 1,
  localparam CyclesPerTarfc = ClockFreq / (1_000_000_000 / TarfcTime) + 1,
  localparam CyclesPerTrcd  = ClockFreq / (1_000_000_000 / TrcdTime) + 1,
  localparam CyclesPerTmrd  = 2 + 1,            /* 2 Clock Cycles */
  localparam CounterWidth   = $clog2(ClockFreq)
)(
  /* System Signals */
  input  logic                  i_sys_clk,    /* System Clock Frequency */
  input  logic                  i_dram_clk,   /* PLL Generated DRAM Clock */
  input  logic                  i_rst_n,      /* Sync Active Low Reset */
  /* ----- User signals ----- */
  /* Write Port */
  input  logic                  i_wr_req,
  input  logic [IAddrWidth-1:0] i_wr_addr,
  input  logic [DataWidth-1:0]  i_wr_data,
  /* Read Port */
  input  logic                  i_rd_req,
  input  logic [IAddrWidth-1:0] i_rd_addr,
  output logic [DataWidth-1:0]  o_rd_data,
  output logic                  o_rd_rdy,
  /* ----- SDRAM signals ----- */
  output logic [OAddrWidth-1:0] o_dram_addr,  /* Read/Write Address */
  inout  tri   [DataWidth-1:0]  io_dram_data, /* Read/Write Data */
  output logic                  o_dram_ba_0,  /* Bank Address [0] */
  output logic                  o_dram_ba_1,  /* Bank Address [1] */
  output logic                  o_dram_ldqm,  /* Low byte data mask */
  output logic                  o_dram_udqm,  /* High byte data mask */
  output logic                  o_dram_we_n,  /* Write enable */
  output logic                  o_dram_cas_n, /* Column address strobe */
  output logic                  o_dram_ras_n, /* Row address strobe */
  output logic                  o_dram_cs_n,  /* Chip select */
  output logic                  o_dram_clk,   /* DRAM Clock */
  output logic                  o_dram_cke    /* Clock Enable */
);
 
  logic refresh_en, refresh_req, refresh_ack;

  sdram_refresh #(
    .ClockFreq(ClockFreq)
  ) sdram_refresh (
    .i_dram_clk,
    .i_rst_n,
    .i_refresh_en(refresh_en),
    .i_refresh_ack(refresh_ack),
    .o_refresh_req(refresh_req)
  );

  /* dram_cmd_t = {CS, RAS, CAS, WE} */
  typedef enum logic [3:0] {
    CMD_NOP      = 4'b1zzz,
    CMD_RD_RDA   = 4'b0101,
    CMD_WR_WRA   = 4'b0100,
    CMD_ACT      = 4'b0011,
    CMD_PRE_PALL = 4'b0010,
    CMD_MRS      = 4'b0000,
    CMD_REF      = 4'b0001
  } dram_cmd_t;
  
  typedef enum logic [4:0] { /* Decrease this bit width to whatever we need at the end */
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
    RDY_NOP,
    EXEC_REF,
    EXEC_WRITE_ACT,
    EXEC_WRITE_WAIT_TRCD,
    EXEC_WRITE_WRITE,
    EXEC_READ_ACT,
    EXEC_READ_WAIT_TRCD,
    EXEC_READ_READ,
    EXEC_READ_WAIT_CAS,
    EXEC_READ_SAMPLE
  } dram_states_t;

  dram_states_t curr_state, next_state;

  logic [CounterWidth-1:0] counter;

  logic counter_rst_n;
  
  logic [3:0] cmd;
  logic write_enable;
  logic [DataWidth-1:0] dram_data_out;

  assign o_dram_cke      = 1'b1;
  assign {o_dram_cs_n, o_dram_ras_n, o_dram_cas_n, o_dram_we_n} = cmd;
  assign o_dram_clk = i_dram_clk;

  // State controller
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n) curr_state <= INIT_RESET;
    else          curr_state <= next_state;

  // Next State Logic Controller
  always_comb begin
    next_state = 'x;
    unique case (curr_state)

      INIT_RESET:                                              next_state = INIT_WAIT;
   
      INIT_WAIT:            if (counter == CyclesPerWait - 1)  next_state = INIT_PALL;
                            else                               next_state = INIT_WAIT;  // @ loopback
      INIT_PALL:                                               next_state = INIT_WAIT_TRP;
 
      INIT_WAIT_TRP:        if (counter == CyclesPerTrp - 1)   next_state = INIT_REF_1; // @ loopback
                            else                               next_state = INIT_WAIT_TRP;
 
      INIT_REF_1:                                              next_state = INIT_WAIT_TARFC_1;
 
      INIT_WAIT_TARFC_1:    if (counter == CyclesPerTarfc - 1) next_state = INIT_REF_2;
                            else                               next_state = INIT_WAIT_TARFC_1; // @ loopback
      INIT_REF_2:                                              next_state = INIT_WAIT_TARFC_2;
       
      INIT_WAIT_TARFC_2:    if (counter == CyclesPerTarfc - 1) next_state = INIT_MRS;
                            else                               next_state = INIT_WAIT_TARFC_2; // @ loopback
 
 
      INIT_MRS:                                                next_state = INIT_WAIT_TMRD;
 
      INIT_WAIT_TMRD:       if (counter == CyclesPerTmrd - 1)  next_state = RDY_NOP;
                            else                               next_state = INIT_WAIT_TMRD; // @ loopback
 
      RDY_NOP:              if (refresh_req)                   next_state = EXEC_REF;
                            else if (i_wr_req)                 next_state = EXEC_WRITE_ACT;
                            else if (i_rd_req)                 next_state = EXEC_READ_ACT;
                            else                               next_state = RDY_NOP; // @ loopback
   
      EXEC_REF:                                                next_state = RDY_NOP;
   
      EXEC_WRITE_ACT:                                          next_state = EXEC_WRITE_WAIT_TRCD;
 
      EXEC_WRITE_WAIT_TRCD: if (counter == CyclesPerTrcd - 1)  next_state = EXEC_WRITE_WRITE;
                            else                               next_state = EXEC_WRITE_WAIT_TRCD; // @ loopback
      
      EXEC_WRITE_WRITE:                                        next_state = RDY_NOP;

      EXEC_READ_ACT:                                           next_state = EXEC_READ_WAIT_TRCD;

      EXEC_READ_WAIT_TRCD:  if (counter == CyclesPerTrcd - 1)  next_state = EXEC_READ_READ;
                            else                               next_state = EXEC_READ_WAIT_TRCD; // @ loopback

      EXEC_READ_READ:                                          next_state = EXEC_READ_WAIT_CAS;

      EXEC_READ_WAIT_CAS:   if (counter == CasLatency - 1)     next_state = EXEC_READ_SAMPLE;
                            else                               next_state = EXEC_READ_WAIT_CAS; // @ loopback

      EXEC_READ_SAMPLE:                                        next_state = RDY_NOP;
    endcase
  end

  // FSM Output Controller
  always_ff @(posedge i_dram_clk) begin
    o_rd_rdy <= '0;
    refresh_ack <= '0;
    cmd <= CMD_NOP;
    unique case (next_state)

      INIT_RESET: begin
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_WAIT: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_PALL: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_PRE_PALL;
        refresh_en                 <= 1'b0;
        o_dram_addr                <= {1'b0, 1'b1, 10'b0};
        write_enable               <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_WAIT_TRP: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_REF_1: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_WAIT_TARFC_1: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_REF_2: begin   
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_WAIT_TARFC_2: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_MRS: begin /* Here Mode Reg is set based on o_dram_addr */
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_MRS;
        /* {A[11] <= 0, A[10] <= 0, A[9] <= WB, A[8] <= 0, A[7] <= 0, A[6:4] <= CAS Latency, A[3] <= Burst Type, A[2:0] <= Burst Length} */
        o_dram_addr                <= {1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 3'b011, 1'b0, 3'b000}; 
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      INIT_WAIT_TMRD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      RDY_NOP: begin
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      EXEC_REF: begin
        refresh_ack                <= 1'b1;
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      EXEC_WRITE_ACT: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= i_wr_addr[RowWidth-1:0]; /* {A[0:11] <= Rows} */
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= i_wr_addr[IAddrWidth-1:ColWidth+RowWidth];
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      EXEC_WRITE_WAIT_TRCD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b11;
      end

      EXEC_WRITE_WRITE: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_WR_WRA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[0:7] <= Cols} */
        o_dram_addr                <= {1'b0, 1'b1, 1'b1, 1'b0, i_wr_addr[RowWidth+ColWidth-1:RowWidth]}; 
        write_enable               <= 1'b1;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= i_wr_addr[IAddrWidth-1:ColWidth+RowWidth];
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so we can control the data buffer, DQM Write Latency is 0 cycles */
      end

      EXEC_READ_ACT: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= i_rd_addr[RowWidth-1:0]; /* {A[0:11] <= Rows} */
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= i_rd_addr[IAddrWidth-1:ColWidth+RowWidth];
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* High so we *don't* control the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_WAIT_TRCD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z; /* {A[0:11] <= Rows} */
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* High so we *don't* control the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_READ: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_RD_RDA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[7:0] <= Cols} */
        o_dram_addr                <= {1'b0, 1'b1, 1'b1, 1'b0, i_rd_addr[RowWidth+ColWidth-1:RowWidth]}; 
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= i_rd_addr[IAddrWidth-1:ColWidth+RowWidth];
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* High so we *don't* control the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_WAIT_CAS: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* High so we *don't* control the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_SAMPLE: begin
        o_rd_rdy                   <= 1'b1;
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* High so we *don't* control the data buffer, DQM Read Latency is 2 cycles */
      end

    endcase
  end

  assign io_dram_data = write_enable ? i_wr_data : 'z;
  assign o_rd_data = io_dram_data;

  /* Incrementing Counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      counter <= '0;
    else if (!counter_rst_n)
      counter <= '0;
    else
      counter <= counter + 1'b1;  

endmodule
