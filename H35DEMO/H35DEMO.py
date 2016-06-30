#
# ------------------------------------------------------------
# Copyright (c) All rights reserved
# SiLab, Institute of Physics, University of Bonn
# ------------------------------------------------------------
#
import os,sys,time

import yaml
import basil
from basil.dut import Dut
import logging
logging.getLogger().setLevel(logging.DEBUG)

class H35DEMO():
    def __init__(self,conf=None):
        
        if conf==None:
            conf = os.path.dirname(os.path.abspath(__file__)) + os.sep + "H35DEMO.yaml"
        
        logging.info("Loading configuration file from %s" % conf)
        
        conf = self._preprocess_conf(conf)
        self.dut=Dut(conf)
        self.dut.init()
        self.dut['CCPD_SPI_RX'].reset()
        self.dut['CCPD_SPI_RX'].set_en(False)
    
    def _preprocess_conf(self, conf):
        return conf
    def power_fei4(self):
        pass
    def configure(self,karg):
        self.dut["CCPD_CONF_A"]["SPARE0"]=1
        self.dut["CCPD_CONF_A"][""]
        self.dut["CCPD_CONF_A"].write()
    def show(self)

if __name__=="__main__":
    chip = H35DEMO()
    chip.init()