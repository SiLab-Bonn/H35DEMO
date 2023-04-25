/**
 * ------------------------------------------------------------
 * Copyright (c) All rights reserved 
 * SiLab , Physics Institute of Bonn University , All Right 
 * ------------------------------------------------------------
 */
 
`timescale 1ps / 1ps
`default_nettype none

`include "clk_gen.v"

//BASIL includes
`include "gpio/gpio.v"

`include "spi/spi.v"
`include "spi/spi_core.v"
`include "spi/blk_mem_gen_8_to_1_2k.v"
   
`include "sram_fifo/sram_fifo_core.v"
`include "sram_fifo/sram_fifo.v"

`include "fei4_rx/fei4_rx_core.v"
`include "fei4_rx/receiver_logic.v"
`include "fei4_rx/sync_master.v"
`include "fei4_rx/rec_sync.v"
`include "fei4_rx/decode_8b10b.v"
`include "fei4_rx/fei4_rx.v"

`include "utils/flag_domain_crossing.v"
`include "utils/cdc_syncfifo.v"
`include "utils/generic_fifo.v"
`include "utils/cdc_pulse_sync.v"
`include "utils/CG_MOD_pos.v"
`include "utils/clock_divider.v"
`include "utils/fx2_to_bus.v"
`include "utils/reset_gen.v"
`include "utils/bus_to_ip.v"
`include "utils/cdc_reset_sync.v"
`include "utils/pulse_gen_rising.v"
`include "utils/3_stage_synchronizer.v"

`include "pulse_gen/pulse_gen.v"
`include "pulse_gen/pulse_gen_core.v"

`include "tlu/tlu_controller.v"
`include "tlu/tlu_controller_core.v"
`include "tlu/tlu_controller_fsm.v"

`include "gpac_adc_rx/gpac_adc_rx.v"
`include "gpac_adc_rx/gpac_adc_rx_core.v"
`include "gpac_adc_rx/gpac_adc_iobuf.v"

`include "cmd_seq/cmd_seq.v"
`include "cmd_seq/cmd_seq_core.v"

`include "fast_spi_rx/fast_spi_rx.v"
`include "fast_spi_rx/fast_spi_rx_core.v"

//`include "tdc_s3/tdc_s3.v"
//`include "tdc_s3/tdc_s3_core.v"

`include "rrp_arbiter/rrp_arbiter.v"

`ifdef COCOTB_SIM //for simulation
    `include "utils/IDDR_sim.v" 
    `include "utils/ODDR_sim.v" 
    `include "utils/BUFG_sim.v" 
    `include "utils/DCM_sim.v" 
    `include "utils/clock_multiplier.v"
    `include "utils/RAMB16_S1_S9_sim.v"
    `include "utils/IBUFDS_sim.v"
    `include "utils/IBUFGDS_sim.v"
    `include "utils/OBUFDS_sim.v"
`else
    `include "utils/IDDR_s3.v"
    `include "utils/ODDR_s3.v"
`endif



module H35DEMO (
    
    input wire FCLK_IN, // 48MHz
    
    //full speed 
    inout wire [7:0] BUS_DATA,
    input wire [15:0] ADD,
    input wire RD_B,
    input wire WR_B,
    
    //high speed
    inout wire [7:0] FD,
    input wire FREAD,
    input wire FSTROBE,
    input wire FMODE,

    //debug ports
    //output wire [15:0] DEBUG_D,
    //output wire [10:0] MULTI_IO, // Pin 1-11, 12: not connected, 13, 15: DGND, 14, 16: VCC_3.3V
  
    //LED
    output wire [4:0] LED,
    
    //SRAM
    output wire [19:0] SRAM_A,
    inout wire [15:0] SRAM_IO,
    output wire SRAM_BHE_B,
    output wire SRAM_BLE_B,
    output wire SRAM_CE1_B,
    output wire SRAM_OE_B,
    output wire SRAM_WE_B,
     
    //FADC CONFIG
    output ADC_CSN,
    output ADC_SCLK,
    output ADC_SDI,
    input ADC_SD0,

    output ADC_ENC_P,
    output ADC_ENC_N,
    input ADC_DCO_P,
    input ADC_DCO_N,
    input ADC_FCO_P,
    input ADC_FCO_N,

    input [3:0] ADC_OUT_P,
    input [3:0] ADC_OUT_N,
     
    // Triggers
    input wire [2:0] LEMO_RX,
    output wire [2:0] TX, // TX[0] == RJ45 trigger clock output, TX[1] == RJ45 busy output
    input wire RJ45_RESET,
    input wire RJ45_TRIGGER,
     
     //FEI4
     input wire MONHIT,       //DIN1
     input wire DOBOUT,
     output wire CMD_DATA,
     output wire CMD_CLK,

    // CCPD
    input wire CCPD_BSOUT,       //DIN2
    input wire CCPD_ASOUT,       //DIN3
    input wire CCPD_SOUT,       //DIN4
    input wire CCPD_NSOUT,       //DIN5
    output wire CCPD_CK2,        //DOUT0
    output wire CCPD_SIN,       //DOUT1
    output wire CCPD_LD,    //DOUT2
    output wire CCPD_RB,    //DOUT3
    output wire CCPD_CK1,   //DOUT4
    output wire CCPD_BSIN,  //DOUT5
    output wire CCPD_ASIN,  //DOUT6
    output wire CCPD_NSIN,  //DOUT7

    output wire CCPD_INJECTION, //INJ
    output wire [3:0] CCPD_DEBUG,     //DEBUG DOUT9, 10, 11, 12

    // I2C
    inout SDA,
    inout SCL
);


// assignments for SCC_HVCMOS2FE-I4B_V1.0 and SCC_HVCMOS2FE-I4B_V1.1
// CCPD

// Assignments
wire BUS_RST;
(* KEEP = "{TRUE}" *)
wire BUS_CLK;
(* KEEP = "{TRUE}" *)
wire CCPD_SPI_CLK;
(* KEEP = "{TRUE}" *)
wire CLK_40;
wire DATA_CLK;
wire RX_CLK;
wire RX_CLK2X;
wire CLK_LOCKED;
wire ADC_ENC;

wire TDC_OUT, TDC_TRIG_OUT,TDC_TDC_OUT;

// TLU
wire TLU_BUSY; // busy signal to TLU to de-assert trigger
wire TLU_CLOCK;

// LEMO & RJ45 Tx
assign TX[0] = TLU_CLOCK; // trigger clock; also connected to RJ45 output
assign TX[1] = TLU_BUSY; // TLU_BUSY signal; also connected to RJ45 output. Asserted when TLU FSM has accepted a trigger or when CMD FSM is busy. 
assign TX[2] = ~TDC_TDC_OUT;

// ------- RESRT/CLOCK  ------- //
reset_gen ireset_gen(.CLK(BUS_CLK), .RST(BUS_RST));

clk_gen iclkgen(
    .U1_CLKIN_IN(FCLK_IN),
    .U1_RST_IN(1'b0),
    .U1_CLKIN_IBUFG_OUT(),
    .U1_CLK0_OUT(BUS_CLK),    // DCM1: 48MHz USB/SRAM clock
    .U1_CLKDV_OUT(ADC_ENC),   // DCM1: 9.6MHz adc
    .U1_STATUS_OUT(),
    .U2_CLKFX_OUT(CLK_40),    // DCM2: 40MHz FEI4 command clock
    .U2_CLKDV_OUT(DATA_CLK),  // DCM2: 16MH FEI4 data clock
    .U2_CLK0_OUT(RX_CLK),     // DCM2: 160MHz data clock ADC clock
    .U2_CLK90_OUT(),
    .U2_CLK2X_OUT(RX_CLK2X),  // DCM2: 320MHz data recovery clock
    .U2_LOCKED_OUT(CLK_LOCKED),
    .U2_STATUS_OUT()
);

// -------  MODULE ADREESSES  ------- //
localparam CMD_BASEADDR = 16'h0000;
localparam CMD_HIGHADDR = 16'h8000-1;

localparam FIFO_BASEADDR = 16'h8100;
localparam FIFO_HIGHADDR = 16'h8200-1;

localparam TLU_BASEADDR = 16'h8200;
localparam TLU_HIGHADDR = 16'h827F;

localparam RX4_BASEADDR = 16'h8300;
localparam RX4_HIGHADDR = 16'h8400-1;

localparam GPIO_RX_BASEADDR = 16'h8800;
localparam GPIO_RX_HIGHADDR = 16'h883F;

//ADC
localparam ADC_SPI_BASEADDR = 16'h8840;                 // 0x8840
localparam ADC_SPI_HIGHADDR = ADC_SPI_BASEADDR + 47;    // 0x886f

localparam ADC_RX_CH0_BASEADDR = 16'h8280;   
localparam ADC_RX_CH0_HIGHADDR = ADC_RX_CH0_BASEADDR + 31; // 
localparam ADC_RX_CH1_BASEADDR = 16'h82A0; 
localparam ADC_RX_CH1_HIGHADDR = ADC_RX_CH1_BASEADDR + 31; // 
localparam ADC_RX_CH2_BASEADDR = 16'h82C0;
localparam ADC_RX_CH2_HIGHADDR = ADC_RX_CH2_BASEADDR + 31; //
localparam ADC_RX_CH3_BASEADDR = 16'h82E0; 
localparam ADC_RX_CH3_HIGHADDR = ADC_RX_CH3_BASEADDR + 31; //

localparam GPIO_TH_BASEADDR = 16'h8890;
localparam GPIO_TH_HIGHADDR = 16'h88bf;

// CCPD
localparam CCPD_PULSE_INJ_BASEADDR= 16'h88B0;
localparam CCPD_PULSE_INJ_HIGHADDR= CCPD_PULSE_INJ_BASEADDR + 15;

localparam CCPD_PULSE_GATE_BASEADDR= 16'h88C0;
localparam CCPD_PULSE_GATE_HIGHADDR= CCPD_PULSE_GATE_BASEADDR+15;

localparam CCPD_SPI_BASEADDR = 16'h8900;
localparam CCPD_SPI_HIGHADDR = 16'h8Aff;

localparam CCPD_SPI_A_BASEADDR = 16'h8B00;
localparam CCPD_SPI_A_HIGHADDR = 16'h8Cff;

localparam CCPD_SPI_B_BASEADDR = 16'h8D00;
localparam CCPD_SPI_B_HIGHADDR = 16'h8Eff;

localparam CCPD_SPI_N_BASEADDR = 16'h8F00;
localparam CCPD_SPI_N_HIGHADDR = 16'h90ff;

localparam CCPD_GPIO_SW_BASEADDR = 16'h9100;
localparam CCPD_GPIO_SW_HIGHADDR = 16'h911f;

localparam CCPD_TDC_BASEADDR = 16'h8870;
localparam CCPD_TDC_HIGHADDR = 16'h888f;

localparam CCPD_RX_ADDR_BASEADDR = 16'h88e0;
localparam CCPD_RX_ADDR_HIGHADDR = 16'h88ff;

localparam CCPD_RX_TIMESTAMP_BASEADDR = 16'h88d0;
localparam CCPD_RX_TIMESTAMP_HIGHADDR = 16'h88ef;

//localparam CCPD_PULSE_SYNC_BASEADDR= 16'h88d0;
//localparam CCPD_PULSE_SYNC_HIGHADDR= CCPD_PULSE_SYNC_BASEADDR + 15;


// -------  BUS SYGNALING  ------- //
wire [15:0] BUS_ADD;
wire BUS_RD, BUS_WR;
fx2_to_bus i_fx2_to_bus (
    .ADD(ADD),
    .RD_B(RD_B),
    .WR_B(WR_B),

    .BUS_CLK(BUS_CLK),
    .BUS_ADD(BUS_ADD),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR)
);

// -------  USER MODULES  ------- //
wire FIFO_NOT_EMPTY; // raised, when SRAM FIFO is not empty
wire FIFO_FULL, FIFO_NEAR_FULL; // raised, when SRAM FIFO is full / near full
wire FIFO_READ_ERROR; // raised, when attempting to read from SRAM FIFO when it is empty

// FEI4g
wire CMD_EXT_START_FLAG;
wire CMD_EXT_START_ENABLE; // from CMD FSM
wire CMD_READY; // to TLU FSM
wire CMD_START_FLAG; // sending FE command triggered by external devices
wire CMD_EXT_START_FLAG_IN;
reg [3:0] CMD_EXT_START_FLAG_FF;
wire [0:1] CCPD_CMD_EXT_SW;
wire CCPD_PULSE_GATE;
////this does not work
assign CMD_EXT_START_FLAG_IN = CCPD_CMD_EXT_SW[1] ? (CCPD_CMD_EXT_SW[0]? CCPD_INJECTION : LEMO_RX[0]):
                                                    (CCPD_CMD_EXT_SW[0]? MONHIT:1'b0);//cmd_seq 
always @ (posedge CLK_40)
begin
    CMD_EXT_START_FLAG_FF <= {CMD_EXT_START_FLAG_FF[2:0],CMD_EXT_START_FLAG_IN};
end
assign CMD_EXT_START_FLAG = CCPD_PULSE_GATE & CMD_EXT_START_FLAG_IN & (~CMD_EXT_START_FLAG_FF[0] | ~CMD_EXT_START_FLAG_FF[1]);

wire TRIGGER_ACCEPTED_FLAG; // from TLU FSM
cmd_seq
#( 
    .BASEADDR(CMD_BASEADDR),
    .HIGHADDR(CMD_HIGHADDR)
) icmd (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    
    .CMD_CLK_OUT(CMD_CLK),
    .CMD_CLK_IN(CLK_40),
     
    .CMD_EXT_START_FLAG(CMD_EXT_START_FLAG),
    .CMD_EXT_START_ENABLE(CMD_EXT_START_ENABLE), //output 

    .CMD_DATA(CMD_DATA),
    .CMD_READY(CMD_READY),
    .CMD_START_FLAG(CMD_START_FLAG) //output
);

parameter DSIZE = 10;
//parameter CLKIN_PERIOD = 6.250;
wire RX_READY, RX_8B10B_DECODER_ERR, RX_FIFO_OVERFLOW_ERR, RX_FIFO_FULL;
wire FE_FIFO_READ;
wire FE_FIFO_EMPTY;
wire [31:0] FE_FIFO_DATA;
fei4_rx
#(
    .BASEADDR(RX4_BASEADDR),
    .HIGHADDR(RX4_HIGHADDR),
    .DSIZE(DSIZE),
    .DATA_IDENTIFIER(4)
) i_fei4_rx (
    .RX_CLK(RX_CLK),
    .RX_CLK2X(RX_CLK2X),
    .DATA_CLK(DATA_CLK),

    .RX_DATA(DOBOUT),

    .RX_READY(RX_READY),
    .RX_8B10B_DECODER_ERR(RX_8B10B_DECODER_ERR),
    .RX_FIFO_OVERFLOW_ERR(RX_FIFO_OVERFLOW_ERR),

    .FIFO_READ(FE_FIFO_READ),
    .FIFO_EMPTY(FE_FIFO_EMPTY),
    .FIFO_DATA(FE_FIFO_DATA),

    .RX_FIFO_FULL(RX_FIFO_FULL),

    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR)
);

wire [3:0] NOT_CONNECTED_RX;
wire ADC_SEL, TLU_SEL,CCPD_RX_SEL,CCPD_TDC_SEL,FE_SEL;
gpio 
#( 
    .BASEADDR(GPIO_RX_BASEADDR),
    .HIGHADDR(GPIO_RX_HIGHADDR),
    .IO_WIDTH(8),
    .IO_DIRECTION(8'hff)
) i_gpio_rx (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    .IO({CCPD_RX_SEL,ADC_SEL,CCPD_TDC_SEL,NOT_CONNECTED_RX[0],TLU_SEL,FE_SEL})
);

wire TLU_FIFO_READ;
wire TLU_FIFO_EMPTY;
wire [31:0] TLU_FIFO_DATA;
wire TLU_FIFO_PEEMPT_REQ;
wire [31:0] TIMESTAMP;
wire TRIGGER_ENABLE; // from CMD FSM
wire TRIGGER_ACKNOWLEDGE_FLAG; // to TLU FSM

tlu_controller #(
    .BASEADDR(TLU_BASEADDR),
    .HIGHADDR(TLU_HIGHADDR),
    .DIVISOR(8)
) i_tlu_controller (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    
    .TRIGGER_CLK(CLK_40),
    
    .FIFO_READ(TLU_FIFO_READ),
    .FIFO_EMPTY(TLU_FIFO_EMPTY),
    .FIFO_DATA(TLU_FIFO_DATA),
    
    .FIFO_PREEMPT_REQ(TLU_FIFO_PEEMPT_REQ),
    
    .TRIGGER({4'b0,CCPD_INJECTION,1'b0,LEMO_RX[0],MONHIT}),
    .TRIGGER_VETO({6'b0,~CCPD_PULSE_GATE,FIFO_FULL}),
     
    .TRIGGER_ACKNOWLEDGE(TRIGGER_ACKNOWLEDGE_FLAG),
    .TRIGGER_ACCEPTED_FLAG(TRIGGER_ACCEPTED_FLAG),
     
    .TLU_TRIGGER(RJ45_TRIGGER),
    .TLU_RESET(RJ45_RESET),
    .TLU_BUSY(TLU_BUSY),
    .TLU_CLOCK(TLU_CLOCK),
    
    .TIMESTAMP(TIMESTAMP)
);
assign TRIGGER_ACKNOWLEDGE_FLAG = CMD_READY; //TODO CMD_READY
//wire TRIGGER_ACCEPTED_FLAG_SYNC;
//cdc_pulse_sync ext_start_sync (.clk_in(CLK_40), .pulse_in(TRIGGER_ACCEPTED_FLAG), .clk_out(SPI_CLK), .pulse_out(TRIGGER_ACCEPTED_FLAG_SYNC)); 

///////////////////////////
// ADC
wire ADC_EN;

    spi 
    #( 
        .BASEADDR(ADC_SPI_BASEADDR), 
        .HIGHADDR(ADC_SPI_HIGHADDR), 
        .MEM_BYTES(2) 
    )  i_spi_adc
    (
         .BUS_CLK(BUS_CLK),
         .BUS_RST(BUS_RST),
         .BUS_ADD(BUS_ADD),
         .BUS_DATA(BUS_DATA),
         .BUS_RD(BUS_RD),
         .BUS_WR(BUS_WR),
         .SPI_CLK(ADC_ENC),
         
         .EXT_START(1'b0),

         .SCLK(ADC_SCLK),
       .SDI(ADC_SDI),
       .SDO(ADC_SD0),
       .SEN(ADC_EN),
       .SLD()
    );
    assign ADC_CSN = !ADC_EN;
    wire [13:0] ADC_IN [3:0];
    wire ADC_DCO, ADC_FCO;
    gpac_adc_iobuf i_gpac_adc_iobuf
    (
        .ADC_CLK(RX_CLK),
          
        .ADC_DCO_P(ADC_DCO_P), .ADC_DCO_N(ADC_DCO_N),
        .ADC_DCO(ADC_DCO),
    
        .ADC_FCO_P(ADC_FCO_P), .ADC_FCO_N(ADC_FCO_N),
        .ADC_FCO(ADC_FCO),
    
        .ADC_ENC(ADC_ENC), 
        .ADC_ENC_P(ADC_ENC_P), .ADC_ENC_N(ADC_ENC_N),
    
        .ADC_IN_P(ADC_OUT_P), .ADC_IN_N(ADC_OUT_N),
        
        .ADC_IN0(ADC_IN[0]), 
        .ADC_IN1(ADC_IN[1]), 
        .ADC_IN2(ADC_IN[2]), 
        .ADC_IN3(ADC_IN[3])
    );

    // GATE
    wire CCPD_ADC_TRIG_GATE,CCPD_ADC_TRIG_GATE_SW;
    reg ccpd_adc_trig_gate_ff;
    always @ (posedge ADC_ENC)
    begin
         ccpd_adc_trig_gate_ff <= CCPD_PULSE_GATE;
    end
    assign CCPD_ADC_TRIG_GATE = CCPD_PULSE_GATE & ~ccpd_adc_trig_gate_ff; //rising edge 

    wire [31:0] FIFO_DATA_ADC [3:0];
    wire [3:0] FIFO_EMPTY_ADC;
    wire [3:0] FIFO_READ_ADC;
    wire [3:0] ADC_ERROR;
    wire ADC_TRIGGER;
	 
    wire [13:0] ADC_TH;
    wire [1:0] ADC_TRIG_TH_SW;
    wire ADC_TRIG_TH;
    reg adc_trig_th_ff;
    gpio #(
        .BASEADDR(GPIO_TH_BASEADDR),
        .HIGHADDR(GPIO_TH_HIGHADDR),
        .IO_WIDTH(16),
        .IO_DIRECTION(16'hffff)
    ) i_gpio_th (
        .BUS_CLK(BUS_CLK), 
        .BUS_RST(BUS_RST), 
        .BUS_ADD(BUS_ADD),
        .BUS_DATA(BUS_DATA),
        .BUS_RD(BUS_RD),
        .BUS_WR(BUS_WR),
        .IO({ADC_TRIG_TH_SW, ADC_TH})
    );
    always@(posedge ADC_ENC)
         adc_trig_th_ff <= ADC_IN[ADC_TRIG_TH_SW] > ADC_TH;
    assign ADC_TRIG_TH = ADC_IN[ADC_TRIG_TH_SW] > ADC_TH && adc_trig_th_ff == 0;

    assign ADC_TRIGGER = (CCPD_ADC_TRIG_GATE_SW) ? CCPD_ADC_TRIG_GATE : ADC_TRIG_TH;

    genvar i;
    generate
    for (i = 0; i < 4; i = i + 1) begin: adc_gen
        gpac_adc_rx 
        #(
            .BASEADDR(ADC_RX_CH0_BASEADDR+32*i), 
            .HIGHADDR(ADC_RX_CH0_HIGHADDR+32*i),
            .ADC_ID(i), 
            .HEADER_ID(1'b1) 
        ) i_gpac_adc_rx
        (
            .ADC_ENC(ADC_ENC),
            .ADC_IN(ADC_IN[i]),

            .ADC_SYNC(ADC_TRIGGER),
            .ADC_TRIGGER(ADC_TRIGGER),

            .BUS_CLK(BUS_CLK),
            .BUS_RST(BUS_RST),
            .BUS_ADD(BUS_ADD),
            .BUS_DATA(BUS_DATA),
            .BUS_RD(BUS_RD),
            .BUS_WR(BUS_WR), 
                
            .FIFO_READ(FIFO_READ_ADC[i]),
            .FIFO_EMPTY(FIFO_EMPTY_ADC[i]),
            .FIFO_DATA(FIFO_DATA_ADC[i]),

            .LOST_ERROR(ADC_ERROR[i])
        );
      end
        endgenerate
        
//////////////////////
// CCPD
assign CCPD_RB=1'b0;
wire [4:0] NC_CCPD_GPIO;
gpio 
#( 
    .BASEADDR(CCPD_GPIO_SW_BASEADDR),
    .HIGHADDR(CCPD_GPIO_SW_HIGHADDR),
    .IO_WIDTH(8),
    .IO_DIRECTION(8'hff)
) i_gpio_ccpd_sw (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    .IO({NC_CCPD_GPIO,CCPD_CMD_EXT_SW,CCPD_ADC_TRIG_GATE_SW})
);

wire CCPD_GATE_EXT_START;
assign CCPD_GATE_EXT_START = 1'b0;
pulse_gen
#( 
    .BASEADDR(CCPD_PULSE_GATE_BASEADDR), 
    .HIGHADDR(CCPD_PULSE_GATE_HIGHADDR)
) i_pulse_gen_gate (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),

    .PULSE_CLK(ADC_ENC),
    .EXT_START(CCPD_GATE_EXT_START),
    .PULSE(CCPD_PULSE_GATE)
);

pulse_gen
#( 
    .BASEADDR(CCPD_PULSE_INJ_BASEADDR), 
    .HIGHADDR(CCPD_PULSE_INJ_HIGHADDR)
) i_pulse_gen_inj (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),

    .PULSE_CLK(ADC_ENC),
    .EXT_START(CCPD_PULSE_GATE),
    .PULSE(CCPD_INJECTION)
);
// SPIs
wire CCPD_SLD,CCPD_ALD,CCPD_BLD,CCPD_NLD;
wire CCPD_SEN,CCPD_AEN,CCPD_BEN,CCPD_NEN;
wire CCPD_SPI_CLK_CE;
reg [7:0] ccpd_spi_ff;
clock_divider #(   
    .DIVISOR(8) //
) i_clock_divisor_spi (
    .CLK(ADC_ENC), // 10MHz
    .RESET(1'b0),
    .CE(CCPD_SPI_CLK_CE),
    .CLOCK(CCPD_SPI_CLK)
);
always@(posedge ADC_ENC)
begin
    ccpd_spi_ff <= {ccpd_spi_ff[7:0],~CCPD_SPI_CLK};
end
assign CCPD_CK1= CCPD_AEN & (ccpd_spi_ff[2]& ~ccpd_spi_ff[3]);
assign CCPD_CK2= CCPD_AEN & (ccpd_spi_ff[4] & ~ccpd_spi_ff[5]);
assign CCPD_LD = CCPD_ALD; //TODO for 4 matrixes

spi
#(         
    .BASEADDR(CCPD_SPI_BASEADDR), 
    .HIGHADDR(CCPD_SPI_HIGHADDR),
    .MEM_BYTES(187) 
) i_ccpd_spi (
         .BUS_CLK(BUS_CLK),
         .BUS_RST(BUS_RST),
         .BUS_ADD(BUS_ADD),
         .BUS_DATA(BUS_DATA),
         .BUS_RD(BUS_RD),
         .BUS_WR(BUS_WR),
        
    .SPI_CLK(CCPD_SPI_CLK),
    .EXT_START(CCPD_PULSE_GATE),

    .SCLK(),
    .SDI(CCPD_SIN),
    .SDO(CCPD_SOUT),
    .SEN(CCPD_SEN),
    .SLD(CCPD_SLD)
);
spi
#(         
    .BASEADDR(CCPD_SPI_A_BASEADDR), 
    .HIGHADDR(CCPD_SPI_A_HIGHADDR),
    .MEM_BYTES(187) 
) i_ccpd_spi_a (
         .BUS_CLK(BUS_CLK),
         .BUS_RST(BUS_RST),
         .BUS_ADD(BUS_ADD),
         .BUS_DATA(BUS_DATA),
         .BUS_RD(BUS_RD),
         .BUS_WR(BUS_WR),

    .SPI_CLK(CCPD_SPI_CLK),
    .EXT_START(CCPD_PULSE_GATE),

    .SCLK(),
    .SDI(CCPD_ASIN),
    .SDO(CCPD_ASOUT),
    .SEN(CCPD_AEN),
    .SLD(CCPD_ALD)
);
assign CCPD_NLD=1'b0;
assign CCPD_NEN=1'b0;
assign CCPD_NSIN=1'b0;
assign CCPD_BLD=1'b0;
assign CCPD_BEN=1'b0;
assign CCPD_BSIN=1'b0;

//RX
wire FIFO_READ_CCPD_RX_ADDR;
wire FIFO_EMPTY_CCPD_RX_ADDR;
wire [31:0] FIFO_DATA_CCPD_RX_ADDR;
wire CCPD_RX_EN;
assign CCPD_RX_EN=1'b0;
fast_spi_rx
#(
    .BASEADDR(CCPD_RX_ADDR_BASEADDR),
    .HIGHADDR(CCPD_RX_ADDR_HIGHADDR),    
    .IDENTIFIER(4'b0001)
) i_ccpd_fast_spi_rx_addr(
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    
    .SCLK(1'b0),
    .SDI(1'b0),
    .SEN(CCPD_RX_EN),
      
    .FIFO_READ(FIFO_READ_CCPD_RX_ADDR),
    .FIFO_EMPTY(FIFO_EMPTY_CCPD_RX_ADDR),
    .FIFO_DATA(FIFO_DATA_CCPD_RX_ADDR)
    
);
wire FIFO_READ_CCPD_RX_TIMESTAMP;
wire FIFO_EMPTY_CCPD_RX_TIMESTAMP;
wire [31:0] FIFO_DATA_CCPD_RX_TIMESTAMP;
fast_spi_rx
#(
    .BASEADDR(CCPD_RX_TIMESTAMP_BASEADDR),
    .HIGHADDR(CCPD_RX_TIMESTAMP_HIGHADDR),     
    .IDENTIFIER(4'b0010)
) i_ccpd_fast_spi_rx_timestamp(
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),
    
    .SCLK(1'b0),
    .SDI(1'b0),
    .SEN(CCPD_RX_EN),
      
    .FIFO_READ(FIFO_READ_CCPD_RX_TIMESTAMP),
    .FIFO_EMPTY(FIFO_EMPTY_CCPD_RX_TIMESTAMP),
    .FIFO_DATA(FIFO_DATA_CCPD_RX_TIMESTAMP)  
);

// TDC
/*wire CCPD_TDC_FIFO_READ,CCPD_TDC_FIFO_EMPTY;
wire [31:0] CCPD_TDC_FIFO_DATA;
tdc_s3
#(
    .BASEADDR(CCPD_TDC_BASEADDR),
    .HIGHADDR(CCPD_TDC_HIGHADDR),
    .CLKDV(4),
    .DATA_IDENTIFIER(4'b0101)
) i_ccpd_tdc (
    .CLK320(RX_CLK2X),
    .CLK160(RX_CLK),
    .DV_CLK(CLK_40),
    .TDC_IN(CCPD_TDC),
    .TDC_OUT(TDC_TDC_OUT),
     
    .TRIG_IN(LEMO_RX[0]),
    .TRIG_OUT(TDC_TRIG_OUT),

    .FIFO_READ(CCPD_TDC_FIFO_READ),
    .FIFO_EMPTY(CCPD_TDC_FIFO_EMPTY),
    .FIFO_DATA(CCPD_TDC_FIFO_DATA),

    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR),

    .ARM_TDC(CMD_START_FLAG),

    .TIMESTAMP(TIMESTAMP[15:0]),
    .EXT_EN(CCPD_PULSE_GATE) 
);*/

// Arbiter
wire ARB_READY_OUT, ARB_WRITE_OUT;
wire [31:0] ARB_DATA_OUT;
wire [7:0] READ_GRANT;
rrp_arbiter 
#( 
    .WIDTH(8)
) i_rrp_arbiter
(
    .RST(BUS_RST),
    .CLK(BUS_CLK),

    .WRITE_REQ({~FIFO_EMPTY_CCPD_RX_ADDR & CCPD_RX_SEL,~FIFO_EMPTY_CCPD_RX_TIMESTAMP & CCPD_RX_SEL,
                ~FIFO_EMPTY_ADC[3] & ADC_SEL,~FIFO_EMPTY_ADC[2] & ADC_SEL,~FIFO_EMPTY_ADC[1] & ADC_SEL,
                ~FIFO_EMPTY_ADC[0] & ADC_SEL,~FE_FIFO_EMPTY & FE_SEL,~TLU_FIFO_EMPTY & TLU_SEL}),
    .HOLD_REQ({7'b0, TLU_FIFO_PEEMPT_REQ}),
    .DATA_IN({FIFO_DATA_CCPD_RX_ADDR,FIFO_DATA_CCPD_RX_TIMESTAMP,
              FIFO_DATA_ADC[3],FIFO_DATA_ADC[2],FIFO_DATA_ADC[1],FIFO_DATA_ADC[0],
              FE_FIFO_DATA,TLU_FIFO_DATA}),
    .READ_GRANT(READ_GRANT),

    .READY_OUT(ARB_READY_OUT),
    .WRITE_OUT(ARB_WRITE_OUT),
    .DATA_OUT(ARB_DATA_OUT)
);
assign TLU_FIFO_READ = READ_GRANT[0];
assign FE_FIFO_READ = READ_GRANT[1];
assign FIFO_READ_ADC= READ_GRANT[5:2];
assign FIFO_READ_CCPD_RX_TIMESTAMP= READ_GRANT[6];
assign FIFO_READ_CCPD_RX_ADDR= READ_GRANT[7];
// SRAM
wire USB_READ;
assign USB_READ = FREAD & FSTROBE;

sram_fifo 
#(
    .BASEADDR(FIFO_BASEADDR),
    .HIGHADDR(FIFO_HIGHADDR)
) i_out_fifo (
    .BUS_CLK(BUS_CLK),
    .BUS_RST(BUS_RST),
    .BUS_ADD(BUS_ADD),
    .BUS_DATA(BUS_DATA),
    .BUS_RD(BUS_RD),
    .BUS_WR(BUS_WR), 

    .SRAM_A(SRAM_A),
    .SRAM_IO(SRAM_IO),
    .SRAM_BHE_B(SRAM_BHE_B),
    .SRAM_BLE_B(SRAM_BLE_B),
    .SRAM_CE1_B(SRAM_CE1_B),
    .SRAM_OE_B(SRAM_OE_B),
    .SRAM_WE_B(SRAM_WE_B),

    .USB_READ(USB_READ),
    .USB_DATA(FD),

    .FIFO_READ_NEXT_OUT(ARB_READY_OUT),
    .FIFO_EMPTY_IN(!ARB_WRITE_OUT),
    .FIFO_DATA(ARB_DATA_OUT),

    .FIFO_NOT_EMPTY(FIFO_NOT_EMPTY),
    .FIFO_FULL(FIFO_FULL),
    .FIFO_NEAR_FULL(FIFO_NEAR_FULL),
    .FIFO_READ_ERROR(FIFO_READ_ERROR)
);
    
// ------- LEDs  ------- //
parameter VERSION = 0; // all on: 31
//wire SHOW_VERSION;
//
//SRLC16E # (
//    .INIT(16'hF000) // in seconds, MSB shifted first
//) SRLC16E_LED (
//    .Q(SHOW_VERSION),
//    .Q15(),
//    .A0(1'b1),
//    .A1(1'b1),
//    .A2(1'b1),
//    .A3(1'b1),
//    .CE(CE_1HZ),
//    .CLK(CLK_40),
//    .D(1'b0)
//);

// LED assignments
assign LED[0] = 0;
assign LED[1] = 0;
assign LED[2] = 0;
assign LED[3] = 0;
assign LED[4] = 0;

assign CCPD_DEBUG[0] = CCPD_PULSE_GATE; //DOUT 11
assign CCPD_DEBUG[1] = TRIGGER_ACCEPTED_FLAG; //TRIGGER_ACKNOWLEDGE_FLAG; //DOUT13
assign CCPD_DEBUG[2] = CMD_EXT_START_ENABLE; //DOUT14
assign CCPD_DEBUG[3] = CMD_READY; //DOUT15


endmodule
