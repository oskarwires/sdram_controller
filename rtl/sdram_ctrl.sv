module sdram_ctrl #(
  parameter  AddrWidth     = 13,
  parameter  DataWidth     = 16,
  parameter  CasLatency    = 3,
  parameter  ClockFreq     = 133_000_000, /* MHz of DRAM Clk */
  parameter  WaitTime      = 200,         /* Microseconds */
  localparam CyclesPerWait = ClockFreq / (1_000_000 / WaitTime)
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

  assign o_dram_cke = 1'b1;

  /* dram_cmd_t = {CS, RAS, CAS, WE} */
  typedef enum logic [3:0] {
    CMD_NOP      = 4'b0111,
    CMD_RD_RDA   = 4'b0101,
    CMD_WR_WRA   = 4'b0100,
    CMD_ACT      = 4'b0011,
    CMD_PRE_PALL = 4'b0010,
    CMD_MRS      = 4'b0000,
  } dram_cmd_t;
  
  typedef enum logic [3:0] {
    INIT_WAIT,
    INIT_PALL,
    INIT_WAIT_TRP,
    INIT_REF,
    INIT_WAIT_TARFC,
    INIT_MRS,
    INIT_WAIT_TMRD,
    CMD_NOP,
    // ...
  } dram_states_t;

  

endmodule
