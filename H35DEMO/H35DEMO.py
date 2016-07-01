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

    def init(self):
        self.dut.init()
        self.dut['CCPD_CONF_A'].reset()
        self.dut['CCPD_CONF_A'].set_en(False)
        self.dut["CCPD_CONF_A"].set_size(1491)

        self.configure(BLR=15,VNBias=1,VNFB=5,VNLogic=10,VPLoad=10,VNSF=30,VP=30,VPAB=20)
        print self.dut["CCPD_CONF_A"][:]

    def _preprocess_conf(self, conf):
        return conf
    def power_fei4(self):
        pass
    def configure(self,**karg):

        if "BLR" in karg.keys():
            self.dut["CCPD_CONF_A"]["BLR"]=karg["BLR"]
        self.dut["CCPD_CONF_A"]["SPARE6"]=0
        if "VNBias" in karg.keys():
            self.dut["CCPD_CONF_A"]["VNBias"]=karg["VNBias"]
        self.dut["CCPD_CONF_A"]["SPARE7"]=0
        if "VNFB" in karg.keys():
           self.dut["CCPD_CONF_A"]["VNFB"]=karg["VNFB"]
        self.dut["CCPD_CONF_A"]["SPARE8"]=0
        self.dut["CCPD_CONF_A"]["DACOut9"]=0x3F
        self.dut["CCPD_CONF_A"]["SPARE9"]=0
        self.dut["CCPD_CONF_A"]["DACOut10"]=0x3F
        self.dut["CCPD_CONF_A"]["SPARE10"]=0
        self.dut["CCPD_CONF_A"]["DACOut11"]=0x3F
        self.dut["CCPD_CONF_A"]["SPARE11"]=0
        if "VNLogic" in karg.keys():
            self.dut["CCPD_CONF_A"]["VNLogic"]=karg["VNLogic"]
        self.dut["CCPD_CONF_A"]["SPARE12"]=0
        if "VPLoad" in karg.keys():
           self.dut["CCPD_CONF_A"]["VPLoad"]=karg["VPLoad"]
        self.dut["CCPD_CONF_A"]["SPARE13"]=0
        if "VNSF" in karg.keys():
           self.dut["CCPD_CONF_A"]["VNSF"]=karg["VNSF"]
        self.dut["CCPD_CONF_A"]["SPARE14"]=1
        if "VP" in karg.keys():
            self.dut["CCPD_CONF_A"]["VP"]=karg["VP"]
        self.dut["CCPD_CONF_A"]["SPARE15"]=1
        if "VPAB" in karg.keys():
            self.dut["CCPD_CONF_A"]["VPAB"]=karg["VPAB"]
        self.dut["CCPD_CONF_A"]["EnCCPD"]=0
        self.dut["CCPD_CONF_A"].write()
        self.dut["CCPD_CONF_A"].start()
        while not self.dut['CCPD_CONF_A'].is_ready:
            pass 

    def show(self):
        pass

if __name__=="__main__":
    chip = H35DEMO()
    chip.init()
