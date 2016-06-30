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

sys.path.append("../")

from H35DEMO.H35DEMO import H35DEMO

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
        
    @mock.patch('H35DEMO.H35DEMO.H35DEMO._preprocess_conf', autospec=True, side_effect=lambda *args, **kwargs: _preprocess_conf(*args, **kwargs)) #change interface to SiSim
    def test_sr(self, mock_preprocess):
    
        self.dut = H35DEMO()
        self.dut.init()

        ret = self.dut['SRAM'].get_data()
        print ret

        '''
        #reset SPI memory
        self.dut['global_conf'].set_size(8*19)
        self.dut['global_conf'].write()
        while not self.dut['global_conf'].is_ready:
            pass
        self.dut['global_conf'].write()
        while not self.dut['global_conf'].is_ready:
            pass 
            
        self.dut['control']['RESET'] = 1
        self.dut['control'].write()
        self.dut['control']['RESET'] = 0
        self.dut['control'].write()
        
        #global reg
        self.dut['global_conf']['PrmpVbpDac'] = 36
        self.dut['global_conf']['vthin1Dac'] = 255
        self.dut['global_conf']['vthin2Dac'] = 0
        self.dut['global_conf']['PrmpVbnFolDac'] = 0
        self.dut['global_conf']['vbnLccDac'] = 51
        self.dut['global_conf']['compVbnDac'] = 25
        self.dut['global_conf']['preCompVbnDac'] = 50
        self.dut['global_conf']['ColSrEn'].setall(True) #enable programming of all columns
        
        self.dut.write_global()
        self.dut.write_global()
        
        send = self.dut['global_conf'].tobytes()
        rec =  self.dut['global_conf'].get_data(size=19)
        
        self.assertEqual(send,rec)

        self.dut['control']['RESET'] = 0b11
        self.dut['control'].write()
        
        #TODO: check for pixels
        ''' 

        
    def tearDown(self):
        self.dut.close()
        time.sleep(1)
        cocotb_compile_clean()

if __name__ == '__main__':
    unittest.main()

