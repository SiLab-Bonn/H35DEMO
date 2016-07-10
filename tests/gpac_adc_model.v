/**
 * ------------------------------------------------------------
 * Copyright (c) All rights reserved 
 * SiLab, Institute of Physics, University of Bonn
 * ------------------------------------------------------------

 */

module adc_ser_model ( 
    input wire CLK, LOAD,
    input wire [13:0] DATA_IN,
    output wire DATA_OUT
);

reg [13:0] ser_reg;

always@(posedge CLK)
    if(LOAD)
        ser_reg <= DATA_IN;
    else
        ser_reg <= {ser_reg[12:0], 1'b0};
        
assign DATA_OUT = ser_reg[13];

endmodule

module gpac_adc_model ( 
    input wire ADC_ENC,
    output wire ADC_DCO, ADC_FCO,
    output wire [3:0] ADC_DATA,
    input wire [13:0] ADC_CH0, ADC_CH1, ADC_CH2, ADC_CH3
    );
    
    clock_multiplier #( .MULTIPLIER(16) ) i_adc_clock_multiplier(.CLK(ADC_ENC),.CLOCK(ADC_DCO));

    reg [3:0] adc_cnt;

    reg [1:0] adc_rst_syn;
    always@(posedge ADC_DCO) begin
        adc_rst_syn <= {adc_rst_syn[0],ADC_ENC};
    end

    wire adc_rst;
    assign adc_rst = adc_rst_syn[0] & !adc_rst_syn[1] ;

    localparam [3:0] ADC_SYNC_DLY = 0;
    always@(posedge ADC_DCO) begin
        if(adc_rst)
            adc_cnt <= ADC_SYNC_DLY;
        else
            adc_cnt <= adc_cnt + 1;
    end

    assign ADC_FCO = adc_cnt[3];
    //assign ADC_CLK = ADC_ENC;

    wire adc_load;
    assign adc_load = (adc_cnt == 7);

    adc_ser_model i_adc_ser0(.CLK(ADC_DCO), .LOAD(adc_load), .DATA_IN(ADC_CH0), .DATA_OUT(ADC_DATA[0]));
    adc_ser_model i_adc_ser1(.CLK(ADC_DCO), .LOAD(adc_load), .DATA_IN(ADC_CH1), .DATA_OUT(ADC_DATA[1]));
    adc_ser_model i_adc_ser2(.CLK(ADC_DCO), .LOAD(adc_load), .DATA_IN(ADC_CH2), .DATA_OUT(ADC_DATA[2]));
    adc_ser_model i_adc_ser3(.CLK(ADC_DCO), .LOAD(adc_load), .DATA_IN(ADC_CH3), .DATA_OUT(ADC_DATA[3]));
    
endmodule


