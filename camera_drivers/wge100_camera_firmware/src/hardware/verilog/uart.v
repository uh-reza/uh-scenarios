module uart(/*AUTOARG*/
   // Outputs
   uart_ack_o, uart_dat_o, uart_tx,
   // Inputs
   uart_stb_i, uart_wea_i, uart_dat_i, sys_clk_i, sys_rst_i
   );

  input uart_stb_i;
  input uart_wea_i;
  input [31:0] uart_dat_i;
  input sys_clk_i;
  input sys_rst_i;
  
  output uart_ack_o;
  output [31:0] uart_dat_o;
  output uart_tx;

  reg [3:0] bitcount;
  reg [8:0] shifter;
  reg [23:0] bitclk;
  reg [23:0] rate;
  reg uart_tx;
  reg ovf;

  wire busy = |bitcount[3:1];
  wire sending = |bitcount;
  assign uart_dat_o = { 31'h0, busy };

  always @(posedge sys_clk_i)
  begin
    if (sys_rst_i) begin
      uart_tx <= 1'h1;
      /*AUTORESET*/
      // Beginning of autoreset for uninitialized flops
      bitclk <= 24'h0;
      bitcount <= 4'h0;
      ovf <= 1'h0;
      rate <= 24'h0;
      shifter <= 9'h0;
      // End of automatics
    end else begin
      //if (uart_stb_i)
      //  $displayh("uart ", uart_dat_i, " ", uart_wea_i, uart_dat_o);  
      
      // We just got a new byte.
      if (uart_stb_i & uart_wea_i & ~busy) begin
        //$displayh(uart_dat_i);  
        $write("%c",uart_dat_i[7:0]);
        rate <= uart_dat_i[31:8];
        shifter <= { uart_dat_i[7:0], 1'h0 };
        bitcount <= 11;
      end
      
      if (sending) begin
        // We add rate to bitclk each cycle. When it overflows, we shift.
        { ovf, bitclk } <= { 1'h0, bitclk } + { 1'h0, rate };
        if (ovf) begin
          { shifter, uart_tx } <= { 1'h1, shifter };
          bitcount <= bitcount - 1;
        end
      end else begin
        // We are idle
        bitclk <= -1;
        ovf <= 0;
      end
    end
  end

  assign uart_ack_o = uart_stb_i;
endmodule // irom
