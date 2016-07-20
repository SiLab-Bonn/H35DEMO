

`timescale 1ns / 1ps
`default_nettype none

module clk_gen    (U1_CLKIN_IN, 
                   U1_RST_IN,  
                   U1_CLKIN_IBUFG_OUT, 
                   U1_CLK0_OUT,
                   U1_CLKDV_OUT, //9.6MHz				 
                   U1_STATUS_OUT, 
                   U2_CLKFX_OUT, 
                   U2_CLKDV_OUT, //16MHz
                   U2_CLK0_OUT,  //160MHz
                   U2_CLK90_OUT, 
                   U2_CLK2X_OUT, 
                   U2_LOCKED_OUT, 
                   U2_STATUS_OUT);

    input wire U1_CLKIN_IN;
    input wire U1_RST_IN;
   output wire U2_CLKFX_OUT;
   output wire U1_CLKIN_IBUFG_OUT;
   output wire U1_CLK0_OUT;
   output wire [7:0] U1_STATUS_OUT;
   output wire U2_CLKDV_OUT,U1_CLKDV_OUT;
   output wire U2_CLK0_OUT;
   output wire U2_CLK90_OUT;
   output wire U2_CLK2X_OUT;
   output wire U2_LOCKED_OUT;
   output wire [7:0] U2_STATUS_OUT;
   
   wire GND_BIT;
   wire U1_CLKIN_IBUFG;
   wire U1_CLK0_BUF;
   wire U1_LOCKED_INV_IN;
   wire U2_CLKDV_BUF,U1_CLKDV_BUF;
   wire U2_CLKFB_IN; //160
   wire U2_CLKFX_BUF;
   wire U2_CLK0_BUF;
   wire U2_CLK90_BUF;
   wire U2_CLK2X_BUF;
   wire U2_FDS_Q_OUT;
   wire U2_FD1_Q_OUT;
   wire U2_FD2_Q_OUT;
   wire U2_FD3_Q_OUT;
   wire U2_LOCKED_INV_RST;
   wire U2_OR3_O_OUT;
   wire U2_RST_IN;
   wire CLKFX_OUT;
   //// input freq=48MHz
   assign GND_BIT = 0;
   assign U1_CLKIN_IBUFG_OUT = U1_CLKIN_IBUFG;
   assign U2_CLK0_OUT = U2_CLKFB_IN;
   DCM #(.CLK_FEEDBACK("1X"), .CLKDV_DIVIDE(5.0), .CLKFX_DIVIDE(3),  //DV_DIVIDE4->3
         .CLKFX_MULTIPLY(10), .CLKIN_DIVIDE_BY_2("FALSE"), 
         .CLKIN_PERIOD(20.833), .CLKOUT_PHASE_SHIFT("NONE"), 
         .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"), .DFS_FREQUENCY_MODE("LOW"), 
         .DLL_FREQUENCY_MODE("LOW"), .DUTY_CYCLE_CORRECTION("TRUE"), 
         .FACTORY_JF(16'h8080), .PHASE_SHIFT(0), .STARTUP_WAIT("FALSE") ) 
         DCM_INST1 (.CLKFB(U1_CLK0_OUT), 
                  .CLKIN(U1_CLKIN_IN), 
                  .DSSEN(GND_BIT), 
                  .PSCLK(GND_BIT), 
                  .PSEN(GND_BIT), 
                  .PSINCDEC(GND_BIT), 
                  .RST(1'b0), 
                  .CLKDV(U1_CLKDV_BUF), 
                  .CLKFX(CLKFX_OUT), 
                  .CLKFX180(), 
                  .CLK0(U1_CLK0_BUF), 
                  .CLK2X(), 
                  .CLK2X180(), 
                  .CLK90(), 
                  .CLK180(), 
                  .CLK270(), 
                  .LOCKED(U1_LOCKED_INV_IN), 
                  .PSDONE(), 
                  .STATUS(U1_STATUS_OUT[7:0]));
   DCM #( .CLK_FEEDBACK("1X"), .CLKDV_DIVIDE(10.0), .CLKFX_DIVIDE(8), //DV_DIVIDE=10
         .CLKFX_MULTIPLY(2), .CLKIN_DIVIDE_BY_2("FALSE"), 
         .CLKIN_PERIOD(6.250), .CLKOUT_PHASE_SHIFT("NONE"), 
         .DESKEW_ADJUST("SYSTEM_SYNCHRONOUS"), .DFS_FREQUENCY_MODE("LOW"), 
         .DLL_FREQUENCY_MODE("LOW"), .DUTY_CYCLE_CORRECTION("TRUE"), 
         .FACTORY_JF(16'h8080), .PHASE_SHIFT(0), .STARTUP_WAIT("FALSE") ) 
         DCM_INST2 (.CLKFB(U2_CLKFB_IN), 
                  .CLKIN(CLKFX_OUT), 
                  .DSSEN(GND_BIT), 
                  .PSCLK(GND_BIT), 
                  .PSEN(GND_BIT), 
                  .PSINCDEC(GND_BIT), 
                  .RST(!U1_LOCKED_INV_IN), 
                  .CLKDV(U2_CLKDV_BUF), //10M 32M
                  .CLKFX(U2_CLKFX_BUF), //40M
                  .CLKFX180(), 
                  .CLK0(U2_CLK0_BUF), //160M
                  .CLK2X(U2_CLK2X_BUF), 
                  .CLK2X180(), 
                  .CLK90(U2_CLK90_BUF), 
                  .CLK180(), 
                  .CLK270(), 
                  .LOCKED(U2_LOCKED_OUT), 
                  .PSDONE(), 
                  .STATUS(U2_STATUS_OUT[7:0]));
		
   BUFG  U1_CLK0_BUFG_INST (.I(U1_CLK0_BUF), 
                           .O(U1_CLK0_OUT));
	BUFG  U1_CLKDV_BUFG_INST (.I(U1_CLKDV_BUF), 
                            .O(U1_CLKDV_OUT));

   BUFG  U2_CLKDV_BUFG_INST (.I(U2_CLKDV_BUF), 
                            .O(U2_CLKDV_OUT));
   BUFG  U2_CLKFX_BUFG_INST (.I(U2_CLKFX_BUF), 
                            .O(U2_CLKFX_OUT));
   BUFG  U2_CLK0_BUFG_INST (.I(U2_CLK0_BUF), 
                           .O(U2_CLKFB_IN));
   BUFG  U2_CLK90_BUFG_INST (.I(U2_CLK90_BUF), 
                            .O(U2_CLK90_OUT));
   BUFG  U2_CLK2X_BUFG_INST (.I(U2_CLK2X_BUF), 
                            .O(U2_CLK2X_OUT));

endmodule
