import logging

from pybar.fei4_run_base import Fei4RunBase
from pybar.run_manager import RunManager


class InitScan(Fei4RunBase):
    '''Init scan
    '''
    _default_run_conf = {}
    
    def init_dut(self):
        # PWR
        self.dut['V_in'].set_current_limit(4095,unit='raw')
        self.dut['dummy'].set_voltage(2)
        self.dut['dummy'].set_enable(1)
        self.dut['V_in'].set_voltage(2.1)
        self.dut['V_in'].set_enable(1)

        # enabling readout
        self.dut['ENABLE_CHANNEL']['FE'] = 1
        #self.dut['ENABLE_CHANNEL']['TLU'] = 1
        self.dut['ENABLE_CHANNEL'].write()

    def configure(self):
        pass

    def scan(self):
        logging.info('Init run...')

    def analyze(self):
        pass

if __name__ == "__main__":
    RunManager('/home/user/workspace/pyBAR/pyBAR_H35DEMO/pybar/configuration.yaml').run_run(InitScan)
