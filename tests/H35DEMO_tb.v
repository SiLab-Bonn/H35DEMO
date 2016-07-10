/**
 * ------------------------------------------------------------
 * Copyright (c) All rights reserved 
 * SiLab, Institute of Physics, University of Bonn
 * ------------------------------------------------------------

 */

`include "H35DEMO.v"
`include "gpac_adc_model.v"

module tb (
    
    input wire FCLK_IN, 
    
    //full speed 
    inout wire [7:0] BUS_DATA,
    input wire [15:0] ADD,
    input wire RD_B,
    input wire WR_B,
    
    //high speed
    inout wire [7:0] FD,
    input wire FREAD,
    input wire FSTROBE,
    input wire FMODE

);

    wire [19:0] SRAM_A;
    wire SRAM_BHE_B;
    wire SRAM_BLE_B;
    wire SRAM_CE1_B;
    wire SRAM_OE_B;
    wire SRAM_WE_B;

    // Bidirs
    wire [15:0] SRAM_IO;

    wire SR_IN;
    wire GLOBAL_SR_CLK;
    wire GLOBAL_CTR_LD;
    wire GLOBAL_DAC_LD;

    wire PIXEL_SR_CLK;
    wire INJECT;

    wire ADC_ENC, ADC_DCO, ADC_FCO;
    wire [3:0] ADC_OUT;
    
    // Instantiate the Unit Under Test (UUT)
    H35DEMO uut (
        .FCLK_IN(FCLK_IN), 
        
        .BUS_DATA(BUS_DATA),
        .ADD(ADD),
        .RD_B(RD_B),
        .WR_B(WR_B),
        
        .FD(FD),
        .FREAD(FREAD),
        .FSTROBE(FSTROBE),
        .FMODE(FMODE),
        
        .SRAM_A(SRAM_A), 
        .SRAM_IO(SRAM_IO), 
        .SRAM_BHE_B(SRAM_BHE_B), 
        .SRAM_BLE_B(SRAM_BLE_B), 
        .SRAM_CE1_B(SRAM_CE1_B), 
        .SRAM_OE_B(SRAM_OE_B), 
        .SRAM_WE_B(SRAM_WE_B),
        
        .ADC_ENC_P(ADC_ENC),
        .ADC_ENC_N(),
        .ADC_DCO_P(ADC_DCO),
        .ADC_DCO_N(~ADC_DCO),
        .ADC_FCO_P(ADC_FCO),
        .ADC_FCO_N(~ADC_FCO),

        .ADC_OUT_P(ADC_OUT),
        .ADC_OUT_N(~ADC_OUT)
    
       
    );
    
   
    /// SRAM Model
    reg [15:0] sram [1048576-1:0];
    always@(negedge SRAM_WE_B)
        sram[SRAM_A] <= SRAM_IO;
    
    assign SRAM_IO = !SRAM_OE_B ? sram[SRAM_A] : 16'hzzzz;
    
    //ADC Model
    reg [13:0] adc_waveform;
    wire [13:0] ADC_CH0, ADC_CH1, ADC_CH2, ADC_CH3;
    assign ADC_CH0 = adc_waveform + 1030;
    assign ADC_CH1 = adc_waveform/2 + 1020;
    assign ADC_CH2 = adc_waveform/3 + 1010;
    assign ADC_CH3 = adc_waveform/4 + 1000;
    
    gpac_adc_model adc ( 
        .ADC_ENC(ADC_ENC),
        .ADC_DCO(ADC_DCO), 
        .ADC_FCO(ADC_FCO),
        .ADC_DATA(ADC_OUT),
        .ADC_CH0(ADC_CH0), 
        .ADC_CH1(ADC_CH1), 
        .ADC_CH2(ADC_CH2), 
        .ADC_CH3(ADC_CH3)
    );
    
    initial begin
        forever begin
            repeat(200)
                @(posedge ADC_ENC) adc_waveform <= 0;
            
            repeat(10)
                @(posedge ADC_ENC) adc_waveform <= adc_waveform + 20;
                
            repeat(100)
                @(posedge ADC_ENC) adc_waveform <= adc_waveform - 2;
                
        end
    end


    initial begin
        $dumpfile("H35DEMO.vcd");
        $dumpvars(0);
    end

endmodule

