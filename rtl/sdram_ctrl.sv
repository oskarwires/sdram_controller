module sdram_ctrl #(
    parameter AddrWidth  = 13,
    parameter DataWidth  = 16,
    parameter CasLatency = 3
)(
  /* System Signals */
  input  logic                 i_clk,
  input  logic                 i_rst_n,
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

  // Lots of code will go here :)!

endmodule
