`timescale 1ns / 1ps
// TODO: ADD WRITE AND READ BURST SUPPORT (PRETTY EASY TO DO)
// TODO: ADD ASYNC FIFO (NOT AS EASY)
// TODO: ABILITY TO UPDATE MRS ON THE FLY MAYBE?
// TODO: ADD A BUSY SIGNAL UNTIL FIFO IS ADDED, NO WAY FOR USER TO KNOW IF REQUEST ACCEPTED
module sdram_ctrl #(
  parameter        RowWidth       = 12,
  parameter        ColWidth       = 8,
  parameter        BankWidth      = 2,
  parameter        DataWidth      = 16,
  parameter        CasLatency     = 2,
  parameter        ClockFreq      = 100_000_000,  /* MHz of DRAM Clock */
  parameter        WaitTime       = 200,          /* Microseconds */
  parameter        TrpTime        = 20,           /* Nanoseconds */
  parameter        TrcdTime       = 20,           /* Nanoseconds */
  parameter        TarfcTime      = 70,           /* Nanoseconds */
  parameter        CyclesPerTmrd  = 2,            /* Clock Cycles */
  parameter        CyclesPerTrdl  = 2,            /* Clock Cycles */
  parameter        OAddrWidth     = 12,  
  parameter  logic AutoPrecharge  = 0,            // 1 if enabled, 0 if disabled
  /* {BA1, BA0, Col[7:0], Row[11:0]} */
  localparam       IAddrWidth     = BankWidth + ColWidth + RowWidth, 
  localparam       CyclesPerWait  = ClockFreq / (1_000_000 / WaitTime),
  localparam       CyclesPerTrp   = ClockFreq / (1_000_000_000 / TrpTime) + 1, // Add 1 clock cycle for safety :)
  localparam       CyclesPerTarfc = ClockFreq / (1_000_000_000 / TarfcTime) + 1,
  localparam       CyclesPerTrcd  = ClockFreq / (1_000_000_000 / TrcdTime) + 1,
  localparam       CounterWidth   = $clog2(ClockFreq)
)(
  /* System Signals */
  input  logic                  i_sys_clk,    /* System Clock Frequency */
  input  logic                  i_dram_clk,   /* PLL Generated DRAM Clock */
  input  logic                  i_rst_n,      /* Sync Active Low Reset */
  /* ----- User signals ----- */
  /* Write Port */
  input  logic                  i_wr_req,
  input  logic [IAddrWidth-1:0] i_wr_addr,    /* {Bank, Col, Row} */
  input  logic [DataWidth-1:0]  i_wr_data,
  /* Read Port */
  input  logic                  i_rd_req,
  input  logic [IAddrWidth-1:0] i_rd_addr,    /* {Bank, Col, Row} */
  output logic [DataWidth-1:0]  o_rd_data,
  output logic                  o_rd_rdy,
  /* ----- SDRAM signals ----- */
  /* These are IOB Packed */
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

  logic [RowWidth-1:0] open_rows [2**BankWidth]; // Holds the current row open in each bank

  logic [BankWidth-1:0] rd_bank, wr_bank;
  logic [ColWidth-1:0]  rd_col, wr_col;
  logic [RowWidth-1:0]  rd_row, wr_row;

  assign rd_bank = i_rd_addr[IAddrWidth-1:ColWidth+RowWidth];
  assign wr_bank = i_wr_addr[IAddrWidth-1:ColWidth+RowWidth];

  assign rd_col  = i_rd_addr[RowWidth+ColWidth-1:RowWidth];
  assign wr_col  = i_wr_addr[RowWidth+ColWidth-1:RowWidth];

  assign rd_row  = i_rd_addr[RowWidth-1:0];
  assign wr_row  = i_wr_addr[RowWidth-1:0];

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
  
  typedef enum logic [5:0] { /* Decrease this bit width to whatever we need at the end */
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
    EXEC_READ_PRECHARGE,
    EXEC_READ_WAIT_TRP,
    EXEC_READ_ACT,
    EXEC_READ_WAIT_TRCD,
    EXEC_READ_READ,
    EXEC_READ_WAIT_CAS,
    EXEC_READ_SAMPLE,
    EXEC_WRITE_WAIT_TRDL,
    EXEC_WRITE_PRECHARGE
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
    next_state = 'x; // Quartus doesn't like this, but a paper I read suggested this default x statement for
    // catching states without any case statements
    unique case (curr_state)

      INIT_RESET:                                               next_state = INIT_WAIT;
    
      INIT_WAIT:            if (counter == CyclesPerWait - 1)   next_state = INIT_PALL;
                            else                                next_state = INIT_WAIT;             // @ loopback
 
      INIT_PALL:                                                next_state = INIT_WAIT_TRP;
  
      INIT_WAIT_TRP:        if (counter == CyclesPerTrp - 1)    next_state = INIT_REF_1;
                            else                                next_state = INIT_WAIT_TRP;         // @ loopback
  
      INIT_REF_1:                                               next_state = INIT_WAIT_TARFC_1;
  
      INIT_WAIT_TARFC_1:    if (counter == CyclesPerTarfc - 1)  next_state = INIT_REF_2;
                            else                                next_state = INIT_WAIT_TARFC_1;     // @ loopback
 
      INIT_REF_2:                                               next_state = INIT_WAIT_TARFC_2;
        
      INIT_WAIT_TARFC_2:    if (counter == CyclesPerTarfc - 1)  next_state = INIT_MRS;
                            else                                next_state = INIT_WAIT_TARFC_2;     // @ loopback
  
  
      INIT_MRS:                                                 next_state = INIT_WAIT_TMRD;
  
      INIT_WAIT_TMRD:       if (counter == CyclesPerTmrd - 1)   next_state = RDY_NOP;
                            else                                next_state = INIT_WAIT_TMRD;        // @ loopback
  
      RDY_NOP:              if (refresh_req)                    next_state = EXEC_REF;
                            else if (i_wr_req)                  next_state = EXEC_WRITE_ACT;
                            else if (i_rd_req) begin
                              if (rd_row == open_rows[rd_bank]) next_state = EXEC_READ_READ;        // We are reading from an open row, nice! We can skip ACT
                /* NB: I don't wait tRC for same open row read as I assume that our original read took over 60 ns, unless our clock rate gets too high? */
                              else                              next_state = EXEC_READ_PRECHARGE;   // We are reading from a closed row :( got to close (precharge) the open one
                            end else                            next_state = RDY_NOP;               // @ loopback
    
      EXEC_REF:                                                 next_state = RDY_NOP;
    
      EXEC_WRITE_ACT:                                           next_state = EXEC_WRITE_WAIT_TRCD;
  
      EXEC_WRITE_WAIT_TRCD: if (counter == CyclesPerTrcd - 1)   next_state = EXEC_WRITE_WRITE;
                            else                                next_state = EXEC_WRITE_WAIT_TRCD;  // @ loopback
       
      EXEC_WRITE_WRITE:     if (!AutoPrecharge)                 next_state = EXEC_WRITE_WAIT_TRDL;  // If we are not auto precharging, lets precharge manually!
                            else                                next_state = RDY_NOP;
 
      EXEC_WRITE_WAIT_TRDL: if (counter == CyclesPerTrdl - 1)   next_state = EXEC_WRITE_PRECHARGE; 
                            else                                next_state = EXEC_WRITE_WAIT_TRDL;  // @ loopback
 
      EXEC_WRITE_PRECHARGE:                                     next_state = RDY_NOP;
      
      EXEC_READ_PRECHARGE:                                      next_state = EXEC_READ_WAIT_TRP;

      EXEC_READ_WAIT_TRP:   if (counter == CyclesPerTrp - 1)    next_state = EXEC_READ_ACT;
                            else                                next_state = EXEC_READ_WAIT_TRP;    // @ loopback
 
      EXEC_READ_ACT:                                            next_state = EXEC_READ_WAIT_TRCD;
 
      EXEC_READ_WAIT_TRCD:  if (counter == CyclesPerTrcd - 1)   next_state = EXEC_READ_READ;
                            else                                next_state = EXEC_READ_WAIT_TRCD;   // @ loopback
  
      EXEC_READ_READ:                                           next_state = EXEC_READ_WAIT_CAS; 
  
      EXEC_READ_WAIT_CAS:   if (counter == CasLatency - 1)      next_state = EXEC_READ_SAMPLE; 
                            else                                next_state = EXEC_READ_WAIT_CAS;    // @ loopback
  
      EXEC_READ_SAMPLE:                                         next_state = RDY_NOP;
 
    endcase
  end

  /* 
    If a read request is to a new row in the same bank, then the old row is closed. Otherwise, it's kept open
    All these read rows left open are closed when the SDRAM refreshes (when instructed to per our timer)
    We close every row after a write
  */

  // FSM Output Controller
  always_ff @(posedge i_dram_clk) begin
    o_rd_rdy    <= '0;
    refresh_ack <= '0;
    cmd         <= CMD_NOP;
    {o_dram_ldqm, o_dram_udqm} <= 2'b11;
    unique case (next_state)

      INIT_RESET: begin
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        open_rows                  <= '{default: 'z};
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_PALL: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_PRE_PALL;
        refresh_en                 <= 1'b0;
        o_dram_addr                <= {1'b0, 1'b1, 10'b0};
        write_enable               <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TRP: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_REF_1: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TARFC_1: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_REF_2: begin   
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TARFC_2: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_MRS: begin /* Here Mode Reg is set based on o_dram_addr */
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_MRS;
        /* {A[11] <= 0, A[10] <= 0, A[9] <= WB, A[8] <= 0, A[7] <= 0, A[6:4] <= CAS Latency, A[3] <= Burst Type, A[2:0] <= Burst Length} */
        o_dram_addr                <= {1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 3'b010, 1'b0, 3'b000}; 
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TMRD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      RDY_NOP: begin
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      EXEC_REF: begin
        refresh_ack                <= 1'b1;
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      EXEC_WRITE_ACT: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= wr_row; /* {A[0:11] <= Rows} */
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
      end

      EXEC_WRITE_WAIT_TRCD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_WRITE_WRITE: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_WR_WRA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[0:7] <= Cols} */
        o_dram_addr                <= {1'b0, AutoPrecharge, 1'b1, 1'b0, wr_col}; 
        write_enable               <= 1'b1;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so we can control the data buffer, DQM Write Latency is 0 cycles */
      end

      EXEC_WRITE_WAIT_TRDL: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_WRITE_PRECHARGE: begin
        cmd                        <= CMD_PRE_PALL;
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= {1'bz, 1'b0, {10{1'bz}}}; // A[10] = 0 for single bank precharge
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
      end

      EXEC_READ_PRECHARGE: begin
        cmd                        <= CMD_PRE_PALL;
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= {1'bz, 1'b0, {10{1'bz}}}; // A[10] = 0 for single bank precharge
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
      end
     
      EXEC_READ_WAIT_TRP: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_READ_ACT: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= rd_row; /* {A[0:11] <= Rows} */
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_WAIT_TRCD: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_READ: begin
        counter_rst_n              <= 1'b0;
        cmd                        <= CMD_RD_RDA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[7:0] <= Cols} */
        o_dram_addr                <= {1'b0, AutoPrecharge, 1'b1, 1'b0, rd_col}; 
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_WAIT_CAS: begin
        counter_rst_n              <= 1'b1;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_SAMPLE: begin
        o_rd_rdy                   <= 1'b1;
        counter_rst_n              <= 1'b0;
        o_dram_addr                <= 'z;
        write_enable               <= 1'b0;
        refresh_en                 <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        open_rows[rd_bank]         <= rd_row; // Latch the current row to the open row reg to keep track of what we can read again
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

    endcase
  end

  assign io_dram_data = write_enable ? i_wr_data : 'z;

  always_ff @(posedge i_dram_clk)
    if (next_state == EXEC_READ_SAMPLE) o_rd_data <= io_dram_data;

  /* Incrementing Counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      counter <= '0;
    else if (!counter_rst_n)
      counter <= '0;
    else
      counter <= counter + 1'b1;  

endmodule
