module sdram_refresh #(
  parameter  ClockFreq       = 133_000_000,
  parameter  RefreshTime     = 64,   /* In ms */
  parameter  RefreshCycles   = 4096, /* Per RefreshTime */
  localparam ClockPerRefresh = ClockFreq / ((1000 / RefreshTime) * RefreshCycles),
  localparam CounterWidth    = $clog2(ClockPerRefresh)
)(
  input  logic i_dram_clk,
  input  logic i_rst_n,
  input  logic i_refresh_en,
  output logic o_refresh_req
);
  
  logic [CounterWidth-1:0] clock_counter;

  // TODO: Make the request STAY high until input acknowlege is asserted

  always_ff @(posedge i_dram_clk) begin
    if (!i_rst_n) begin
      o_refresh_req <= '0;
      clock_counter <= '0;
    end else if (!i_refresh_en) begin
      o_refresh_req <= '0;
      clock_counter <= '0;
    end else begin
      if (clock_counter == ClockPerRefresh - 1'b1)  begin
          o_refresh_req <= '1;
          clock_counter <= '0;
      end else begin
          o_refresh_req <= '0;
          clock_counter <= clock_counter + 1'b1;
      end
    end 
  end

endmodule
