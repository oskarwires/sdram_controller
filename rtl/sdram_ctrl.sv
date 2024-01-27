`timescale 1ns / 1ps
// TODO: ADD ASYNC FIFO. Also, do we need a FIFO? Obviously on a system with a different main clock frequency than the SDRAM's, but is that common?
module sdram_ctrl #(
  parameter logic       AutoPrecharge    = 0,            // 1 if enabled, 0 if disabled
  parameter             BurstLength      = 4,            // How many values do we read & write? Supported values: 1, 2, 4, 8
  parameter             RowWidth         = 12,
  parameter             ColWidth         = 8,
  parameter             BankWidth        = 2,
  parameter             DataWidth        = 16,
  parameter logic [2:0] CasLatency       = 2,
  parameter             ClockFreq        = 100_000_000,  /* MHz of DRAM Clock */
  parameter             WaitTime         = 200,          /* Microseconds */
  parameter             TrpTime          = 20,           /* Nanoseconds */
  parameter             TrcdTime         = 20,           /* Nanoseconds */
  parameter             TarfcTime        = 70,           /* Nanoseconds */
  parameter             CyclesPerTmrd    = 2,            /* Clock Cycles */
  parameter             CyclesPerTrdl    = 2,            /* Clock Cycles */
  parameter             OAddrWidth       = 12,  
  /* {BA1, BA0, Col[7:0], Row[11:0]} */
  localparam            IAddrWidth       = BankWidth + ColWidth + RowWidth, 
  localparam            CyclesPerWait    = ClockFreq / (1_000_000 / WaitTime),
  localparam            CyclesPerTrp     = ClockFreq / (1_000_000_000 / TrpTime) + 1, // Add 1 clock cycle for safety :)
  localparam            CyclesPerTarfc   = ClockFreq / (1_000_000_000 / TarfcTime) + 1,
  localparam            CyclesPerTrcd    = ClockFreq / (1_000_000_000 / TrcdTime) + 1,
  localparam            CounterWidth     = $clog2(ClockFreq),
  localparam            WordCounterWidth = $clog2(BurstLength),
  localparam            BurstLengthBits  = ConvertBurstLength(BurstLength)
)(
  /* System Signals */
  input  logic                  i_sys_clk,    /* System Clock Frequency */
  input  logic                  i_dram_clk,   /* PLL Generated DRAM Clock */
  input  logic                  i_rst_n,      /* Sync Active Low Reset */
  /* ----- User signals ----- */
  output logic                  o_ready,      /* Is the controller ready for a request? */
  /* Write Port */
  input  logic                  i_wr_req,
  input  logic [IAddrWidth-1:0] i_wr_addr,    /* {Bank, Col, Row} */
  input  logic [DataWidth-1:0]  i_wr_data [BurstLength],
  /* Read Port */
  input  logic                  i_rd_req,
  input  logic [IAddrWidth-1:0] i_rd_addr,    /* {Bank, Col, Row} */
  output logic [DataWidth-1:0]  o_rd_data [BurstLength],
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

  function automatic [2:0] ConvertBurstLength(input integer BurstLength);
    begin
      case (BurstLength)
        1: ConvertBurstLength = 3'b000;
        2: ConvertBurstLength = 3'b001;
        4: ConvertBurstLength = 3'b010;
        8: ConvertBurstLength = 3'b011;
        default: begin
          $error("Invalid burst length. Supported values: 1,2,4,8");
          ConvertBurstLength = 3'bxxx;
        end
      endcase
    end
  endfunction
 
  logic refresh_en, refresh_req, refresh_ack;

  logic [RowWidth:0] open_rows [2**BankWidth]; // Holds the current row open in each bank
  // We do RowWidth and not RowWidth - 1 as we have a valid bit as the MSB. If this bit isn't set, then we assume we haven't opened a row yet
  // and we then do Bank ACTivate.
  // open_rows[i] = {row_valid, row{11:0}}

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
    EXEC_WAIT_TARFC,
    EXEC_PRECHARGE_ALL,
    EXEC_WRITE_PRECHARGE,
    EXEC_WRITE_WAIT_TRP,
    EXEC_WRITE_ACT,
    EXEC_WRITE_WAIT_TRCD,
    EXEC_WRITE_WRITE,
    EXEC_WRITE_FINISH_WRITING,
    EXEC_READ_PRECHARGE,
    EXEC_READ_WAIT_TRP,
    EXEC_READ_ACT,
    EXEC_READ_WAIT_TRCD,
    EXEC_READ_READ,
    EXEC_READ_WAIT_CAS,
    EXEC_READ_SAMPLE
  } dram_states_t;

  dram_states_t curr_state, next_state;

  logic [CounterWidth-1:0] clk_counter;
  logic [WordCounterWidth-1:0] word_counter;

  logic clk_counter_rst_n;
  logic word_counter_rst_n;
  
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
    unique case (curr_state)
      INIT_RESET:                                                       next_state = INIT_WAIT;
              
      INIT_WAIT:            if (clk_counter == CyclesPerWait - 1)       next_state = INIT_PALL;
                            else                                        next_state = INIT_WAIT;                 // @ loopback
               
      INIT_PALL:                                                        next_state = INIT_WAIT_TRP;    
                
      INIT_WAIT_TRP:        if (clk_counter == CyclesPerTrp - 1)        next_state = INIT_REF_1;    
                            else                                        next_state = INIT_WAIT_TRP;             // @ loopback
                
      INIT_REF_1:                                                       next_state = INIT_WAIT_TARFC_1;    
                
      INIT_WAIT_TARFC_1:    if (clk_counter == CyclesPerTarfc - 1)      next_state = INIT_REF_2;    
                            else                                        next_state = INIT_WAIT_TARFC_1;         // @ loopback
               
      INIT_REF_2:                                                       next_state = INIT_WAIT_TARFC_2;    
                      
      INIT_WAIT_TARFC_2:    if (clk_counter == CyclesPerTarfc - 1)      next_state = INIT_MRS;    
                            else                                        next_state = INIT_WAIT_TARFC_2;         // @ loopback
            
      INIT_MRS:                                                         next_state = INIT_WAIT_TMRD;
            
      INIT_WAIT_TMRD:       if (clk_counter == CyclesPerTmrd - 1)       next_state = RDY_NOP;
                            else                                        next_state = INIT_WAIT_TMRD;            // @ loopback
        
      RDY_NOP:     if (refresh_req)                                     next_state = EXEC_REF;
                   else if (i_wr_req)
                     if (AutoPrecharge)                                 next_state = EXEC_WRITE_ACT;
                     else // Here we handle if we should precharge and/or ACT for our write, as we are not autoprecharging
                       if (!open_rows[wr_bank][RowWidth])               next_state = EXEC_WRITE_ACT;
                       else if (wr_row == open_rows[wr_bank][RowWidth-1:0] && open_rows[wr_bank][RowWidth]) // This multiline is a bit ugly... sorry
                                                                        next_state = EXEC_WRITE_WRITE;      // We are writing to an open row in the same bank, nice! We can skip ACT and Precharging
                       else                                             next_state = EXEC_WRITE_PRECHARGE;  // We are writing to a closed row in same bank :( got to close (precharge) the open one and then bank ACTivate  
                   else if (i_rd_req)      
                     if (AutoPrecharge)                                 next_state = EXEC_READ_ACT;
                     else // Here we handle if we should precharge and/or ACT for our read, as we are not autoprecharging
                       if (!open_rows[rd_bank][RowWidth])               next_state = EXEC_READ_ACT; // We haven't read from a row in this bank yet as valid bit isn't set. We can skip precharging and just bank ACTivate
                       else if (rd_row == open_rows[rd_bank][RowWidth-1:0] && open_rows[rd_bank][RowWidth]) // This multiline is also a bit ugly... sorry
                                                                        next_state = EXEC_READ_READ; // We are reading from an open row in the same bank, nice! We can skip ACT and Precharging
                       else                                             next_state = EXEC_READ_PRECHARGE; // We are reading from a closed row in same bank :( got to close (precharge) the open one and then bank ACTivate  
                   else                                                 next_state = RDY_NOP;                   // @ loopback
            
      EXEC_REF:             if (AutoPrecharge)                          next_state = RDY_NOP;
                            else                                        next_state = EXEC_WAIT_TARFC;       // Next we wait tARFC, then we precharge all banks to close open regs
                    
      EXEC_WAIT_TARFC:      if (clk_counter == CyclesPerTarfc - 1)      next_state = EXEC_PRECHARGE_ALL;    // Precharge all banks to close open rows
                            else                                        next_state = EXEC_WAIT_TARFC;           // @ loopback
      
      EXEC_PRECHARGE_ALL:                                               next_state = RDY_NOP;

      EXEC_WRITE_PRECHARGE:                                             next_state = EXEC_WRITE_WAIT_TRP;

      EXEC_WRITE_WAIT_TRP:  if (clk_counter == CyclesPerTrp - 1)        next_state = EXEC_WRITE_ACT;
                            else                                        next_state = EXEC_WRITE_WAIT_TRP;        // @ loopback
                
      EXEC_WRITE_ACT:                                                   next_state = EXEC_WRITE_WAIT_TRCD;
              
      EXEC_WRITE_WAIT_TRCD: if (clk_counter == CyclesPerTrcd - 1)       next_state = EXEC_WRITE_WRITE;
                            else                                        next_state = EXEC_WRITE_WAIT_TRCD;      // @ loopback
                  
      EXEC_WRITE_WRITE:     if (BurstLength == 1)                       next_state = RDY_NOP; 
                            else                                        next_state = EXEC_WRITE_FINISH_WRITING; // If we are writing more than one word, then go to finish writing state
     
      EXEC_WRITE_FINISH_WRITING: if (word_counter == BurstLength - 1)   next_state = RDY_NOP; // This state is for the part of the writing done (for remaining BurstLength - 1 bits) while asserting NOP
                                 else                                   next_state = EXEC_WRITE_FINISH_WRITING; // @ loopback
                  
      EXEC_READ_PRECHARGE:                                              next_state = EXEC_READ_WAIT_TRP;
            
      EXEC_READ_WAIT_TRP:   if (clk_counter == CyclesPerTrp - 1)        next_state = EXEC_READ_ACT;
                            else                                        next_state = EXEC_READ_WAIT_TRP;        // @ loopback
             
      EXEC_READ_ACT:                                                    next_state = EXEC_READ_WAIT_TRCD;
             
      EXEC_READ_WAIT_TRCD:  if (clk_counter == CyclesPerTrcd - 1)       next_state = EXEC_READ_READ;
                            else                                        next_state = EXEC_READ_WAIT_TRCD;       // @ loopback
              
      EXEC_READ_READ:                                                   next_state = EXEC_READ_WAIT_CAS; 
              
      EXEC_READ_WAIT_CAS:   if (clk_counter == CasLatency - 2)          next_state = EXEC_READ_SAMPLE; 
                            else                                        next_state = EXEC_READ_WAIT_CAS;        // @ loopback
              
      EXEC_READ_SAMPLE:     if (word_counter == BurstLength - 1)        next_state = RDY_NOP;
                            else                                        next_state = EXEC_READ_SAMPLE;          // @ loopback
 
    endcase
  end

  /* 
    If a read / write request is to a new row in the same bank, then the old row is closed. Otherwise, it's kept open
    All these read / write rows left open are closed during an all bank precharged, which we do after every refresh cycle (4096 times every 64 ms or every ~15.6 us).
    This is done by an all bank precharge immediatelly after the refresh command
  */

  // FSM Output Controller
  always_ff @(posedge i_dram_clk) begin
    refresh_ack        <= '0;
    cmd                <= CMD_NOP;
    word_counter_rst_n <= '0;
    refresh_en         <= 1'b1;
    write_enable       <= 1'b0;
    o_ready            <= 1'b0;
    {o_dram_ldqm, o_dram_udqm} <= 2'b11;
    unique case (next_state)

      INIT_RESET: begin
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= '0;
        refresh_en                 <= 1'b0;
        open_rows                  <= '{default: '0};
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT: begin
        clk_counter_rst_n           <= 1'b1;
        o_dram_addr                <= 'z;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_PALL: begin
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_PRE_PALL;
        refresh_en                 <= 1'b0;
        o_dram_addr                <= {1'b0, 1'b1, 10'b0};
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TRP: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_REF_1: begin
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TARFC_1: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_REF_2: begin   
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      INIT_WAIT_TARFC_2: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      INIT_MRS: begin /* Here Mode Reg is set based on o_dram_addr */
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_MRS;
        /* {A[11] <= 0, A[10] <= 0, A[9] <= WB, A[8] <= 0, A[7] <= 0, A[6:4] <= CAS Latency, A[3] <= Burst Type, A[2:0] <= Burst Length} */
        // WB = 1 if Single Location Access, WB = 0 if Programmed Burst Length
        o_dram_addr                <= {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, CasLatency, 1'b0, BurstLengthBits}; 
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
        open_rows                  <= '{default: '0}; // Init open rows to all 0, no valid bit
      end

      INIT_WAIT_TMRD: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        refresh_en                 <= 1'b0;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      RDY_NOP: begin
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= 'z;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
        o_ready                    <= 1'b1;
      end

      EXEC_REF: begin
        refresh_ack                <= 1'b1;
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_REF;
        o_dram_addr                <= '0;
        {o_dram_ba_0, o_dram_ba_1} <= 2'b00;
      end

      EXEC_WAIT_TARFC: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_0, o_dram_ba_1} <= 'z;
      end

      EXEC_PRECHARGE_ALL: begin
        open_rows                  <= '{default: '0}; // Reset all open rows, as we close them here (all bank precharge)
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_PRE_PALL;
        o_dram_addr                <= {1'bz, 1'b1, {10{1'bz}}}; // A[10] = 1 for all bank precharge
        {o_dram_ba_1, o_dram_ba_0} <= 2'b00;
      end

      EXEC_WRITE_PRECHARGE: begin
        cmd                        <= CMD_PRE_PALL;
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= {1'bz, 1'b0, {10{1'bz}}}; // A[10] = 0 for single bank precharge
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
      end

      EXEC_WRITE_WAIT_TRP: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_WRITE_ACT: begin
        clk_counter_rst_n         <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= wr_row; /* {A[0:11] <= Rows} */
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
      end

      EXEC_WRITE_WAIT_TRCD: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_WRITE_WRITE: begin
        word_counter_rst_n         <= 1'b1;
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_WR_WRA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[0:7] <= Cols} */
        o_dram_addr                <= {1'b0, AutoPrecharge, 1'b1, 1'b0, wr_col}; 
        write_enable               <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= wr_bank;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so we can control the data buffer, DQM Write Latency is 0 cycles */
        open_rows[wr_bank]         <= {1'b1, wr_row}; // Latch the current row to the open row reg to keep track of what we can read/write to again
      end

      EXEC_WRITE_FINISH_WRITING: begin // We finish writing the BurstLength - 1 bits left, while no command is asserted
        word_counter_rst_n         <= 1'b1;
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= 'z; 
        write_enable               <= 1'b1;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so we can control the data buffer, DQM Write Latency is 0 cycles */
      end

      EXEC_READ_PRECHARGE: begin
        cmd                        <= CMD_PRE_PALL;
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= {1'bz, 1'b0, {10{1'bz}}}; // A[10] = 0 for single bank precharge
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
      end
     
      EXEC_READ_WAIT_TRP: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_READ_ACT: begin
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_ACT;
        o_dram_addr                <= rd_row; /* {A[0:11] <= Rows} */
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
      end

      EXEC_READ_WAIT_TRCD: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
      end

      EXEC_READ_READ: begin
        clk_counter_rst_n          <= 1'b0;
        cmd                        <= CMD_RD_RDA;
        /* {A[11] <= ?, A[10] <= Auto Precharge, A[9] <= Single Write, A[8] <= ?,  A[7:0] <= Cols} */
        o_dram_addr                <= {1'b0, AutoPrecharge, 1'b1, 1'b0, rd_col}; 
        {o_dram_ba_1, o_dram_ba_0} <= rd_bank;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_WAIT_CAS: begin
        clk_counter_rst_n          <= 1'b1;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

      EXEC_READ_SAMPLE: begin // We read from io_dram_data here
        word_counter_rst_n         <= 1'b1;
        clk_counter_rst_n          <= 1'b0;
        o_dram_addr                <= 'z;
        {o_dram_ba_1, o_dram_ba_0} <= 'z;
        open_rows[rd_bank]         <= {1'b1, rd_row}; // Latch the current row to the open row reg to keep track of what we can read/write to again
        {o_dram_ldqm, o_dram_udqm} <= 2'b00; /* Low so the SDRAM controls the data buffer, DQM Read Latency is 2 cycles */
      end

    endcase
  end

  assign io_dram_data = write_enable ? i_wr_data[word_counter] : 'z; // Tri-state buffer

  always_ff @(posedge i_sys_clk) // Here is what we use to sample the io_dram_data
    if (curr_state == EXEC_READ_SAMPLE) o_rd_data[word_counter] = io_dram_data;

  always_ff @(posedge i_sys_clk) // Here is where we output the o_rd_rdy signal
    if (curr_state == EXEC_READ_SAMPLE) // I couldn't figure out how to do this inside the always_ff for the FSM outputs, so here it is!
      if (word_counter == BurstLength - 1) o_rd_rdy <= 1'b1;
      else                                 o_rd_rdy <= 1'b0;
    else                                   o_rd_rdy <= 1'b0;

  /* Incrementing word counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      word_counter <= '0;
    else if (!word_counter_rst_n)
      word_counter <= '0;
    else
      word_counter <= word_counter + 1'b1;

  /* Incrementing clock counter */
  always_ff @(posedge i_dram_clk)
    if (!i_rst_n)
      clk_counter <= '0;
    else if (!clk_counter_rst_n)
      clk_counter <= '0;
    else
      clk_counter <= clk_counter + 1'b1;  

endmodule
