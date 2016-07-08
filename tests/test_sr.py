#
# ------------------------------------------------------------
# Copyright (c) All rights reserved
# SiLab, Institute of Physics, University of Bonn
# ------------------------------------------------------------
#

import unittest
import os
from basil.utils.sim.utils import cocotb_compile_and_run, cocotb_compile_clean
import sys
import yaml
import mock
import time

sys.path.append("../H35DEMO")

from H35DEMO import H35DEMO

def _preprocess_conf(self, conf):
    
    with open(conf, 'r') as f:
        cnfg = yaml.load(f)
    
    cnfg['transfer_layer'][0]['type'] = 'SiSim'
    cnfg['hw_drivers'][0]['no_calibration'] = True

    return cnfg
    
class TestSimSr(unittest.TestCase):

    def setUp(self):
        
        proj_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__))) #../
        
        cocotb_compile_and_run(
            sim_files = [proj_dir + '/tests/H35DEMO_tb.v'], 
            sim_bus = 'basil.utils.sim.SiLibUsbBusDriver',
            include_dirs = (proj_dir, proj_dir + "/firmware/src"),
        )
        
    @mock.patch('H35DEMO.H35DEMO._preprocess_conf', autospec=True, side_effect=lambda *args, **kwargs: _preprocess_conf(*args, **kwargs)) #change interface to SiSim
    def test_sr(self, mock_preprocess):
    
        self.chip = H35DEMO()
        #H35DEMO.init()
        self.chip.dut.init()
        self.chip.dut['dummy'].set_voltage(2)
        self.chip.dut['dummy'].set_current_limit(0.1)
        self.chip.dut['dummy'].set_enable(1)
        self.chip.dut['CCPD_CONF_A'].reset()
        self.chip.dut['CCPD_CONF_A'].set_en(False)
        self.chip.dut['CCPD_CONF_A'].set_size(1491)
        
        #self.chip.configure(BLR=15,VNBias=1,VNFB=5,VNLogic=10,VPLoad=10,VNSF=30,VP=30,VPAB=20)
        
        #### test injection
        #self.chip.set_inj_all(inj_width=10,inj_n=10)
        #self.chip.inject()
        #while not self.chip.dut['CCPD_INJ']['READY']:
        #    pass

        #### test gpio
        #self.chip.dut["CCPD_ADC_TH"]["SW"]=1
        #self.chip.dut["CCPD_ADC_TH"]["VALUE"]=0x0001
        #self.chip.dut["CCPD_ADC_TH"].write()

        #### test adc 
        self.chip.set_adc_th()
        self.chip.set_adc()
        #self.chip.get_adc_data() ## fail at dut['sram'].get_data()
        if self.chip.dut["fadc0_rx"].get_en_trigger()==0:
                self.chip.dut['sram'].reset()
                for i in range(4):
                    self.chip.dut['fadc%d_rx'%i].start()
                flg=0
        elif self.chip.dut["CCPD_SW"]["ADC_TRIG_GATE"]==1: ## inj TODO exclude extrig of gate==0 (tlu)
                self.chip.dut['sram'].reset()
                self.chip.dut['CCPD_GATE'].start()
                flg=1
        else:
                flg=2  ## th TODO tlu
        while (flg==0 and self.chip.dut['fadc0_rx'].is_done()==0):
             pass
        while (flg==0 and self.chip.dut['fadc1_rx'].is_done()==0):
             pass
        while (flg==0 and self.chip.dut['fadc2_rx'].is_done()==0):
             pass
        while (flg==0 and self.chip.dut['fadc3_rx'].is_done()==0):
             pass

        #### test sram.get_data() ## this passes
        #ret = self.chip.dut['sram'].get_data()
        #print ret
        
    def tearDown(self):
        self.chip.dut.close()
        time.sleep(1)
        cocotb_compile_clean()

if __name__ == '__main__':
    unittest.main()

