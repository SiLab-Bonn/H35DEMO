#
# ------------------------------------------------------------
# Copyright (c) All rights reserved
# SiLab, Institute of Physics, University of Bonn
# ------------------------------------------------------------
#
import os,sys,time
import numpy as np

import basil
from basil.dut import Dut
import logging
logging.getLogger().setLevel(logging.DEBUG)

class Log():
    def __init__(self,logfile='H35DEMO.log'):

        self.logfile=logfile
    def info(self,s):
        print s
        s.replace("\n","")
        with open(self.logfile,"a") as f:
            f.write("%s %s\n"%(time.strftime("%Y/%m/%d-%H:%M:%S"),s))
    def archive(self):
        with open('archive_%s'%self.logfile, 'a') as fo:
            try:
                with open(self.logfile) as f:
                    for line in f:
                        fo.write(line)
                with open(self.logfile,"w") as f:
                    f.write("")
            except:
                pass
    def show(self,dat):
        ### TODO format nicely
        print "CCPD_CONF_A:dummy=",dat['CCPD_CONF_A'][:80]
        print "           :col=",dat['CCPD_CONF_A'][80:1280]
        print "           :row=",dat['CCPD_CONF_A'][1280:1280+23]
        print "           :dac=",dat['CCPD_CONF_A'][1280+23:]
        print "Injection:dealy=%d,width=%d,repeat=%d,en=%d,voltages=%s"%(dat['CCPD_INJ']["DELAY"],dat['CCPD_INJ']["WIDTH"]
                    ,dat['CCPD_INJ']["REPEAT"],dat['CCPD_INJ']["EN"],dat['CCPD_INJ_VOLTAGE'])
        with open(self.logfile,"a") as f:
            f.write("%s %s: %s\n"%(time.strftime("%Y/%m/%d-%H:%M:%S"),'CCPD_CONF_A',dat['CCPD_CONF_A']))
            f.write("%s %s: %s\n"%(time.strftime("%Y/%m/%d-%H:%M:%S"),'CCPD_GATE',dat['CCPD_GATE']))  
            f.write("%s %s: %s\n"%(time.strftime("%Y/%m/%d-%H:%M:%S"),'CCPD_INJ',dat['CCPD_INJ']))   
            f.write("%s %s: %s\n"%(time.strftime("%Y/%m/%d-%H:%M:%S"),'CCPD_INJ_VOLTAGE',dat['CCPD_INJ_VOLTAGE'])) 

class H35DEMO():
    def __init__(self,conf='H35DEMO.yaml'):
        #if conf==None:
        #    conf = os.path.dirname(os.path.abspath(__file__)) + os.sep + "H35DEMO.yaml"
        
        logging.info("Loading configuration file from %s" % conf)
        conf = self._preprocess_conf(conf)
        self.dut=Dut(conf)
        self.logger=Log()

    def init(self):
        self.dut.init()
        self.dut['dummy'].set_voltage(2)
        self.dut['dummy'].set_current_limit(0.1)
        self.dut['dummy'].set_enable(1)
        self.dut['CCPD_CONF_A'].reset()
        self.dut['CCPD_CONF_A'].set_en(False)
        self.dut['CCPD_CONF_A'].set_size(1491)

        self.configure(BLR=15,VNBias=1,VNFB=5,VNLogic=10,VPLoad=10,VNSF=30,VP=30,VPAB=20)
        self.set_inj_all(inj_high=1,inj_low=0,inj_n=1,inj_width=500)

    def _preprocess_conf(self, conf):
        return conf
    def set_adc_th(self,th=6500,ref=1.5,ch=0):
        # set th
        self.dut["CCPD_ADC_TH"]["VALUE"]= th
        self.dut["CCPD_ADC_TH"]["SW"]=ch
        self.dut['CCPD_ADC_TH'].write()

        if not isinstance(ref, list):
            ref=[ref]*4
        for i in range(4):
            self.dut['ADCref%d'%i].set_voltage(ref[i], unit='V')
        self.logger.info("set_adc_th th=%d ref=%s ch=%d"%(th,str(ref),ch))

    def set_adc(self,adc_count=500,adc_delay=20,extrig="none"):
        # set gpio  TODO
        self.dut['ENABLE_CHANNEL']['ADC'] = 1
        if extrig=="tlu":
            self.dut['ENABLE_CHANNEL']['TLU'] = 1
        else:
            self.dut['ENABLE_CHANNEL']['TLU'] = 0
        self.dut['ENABLE_CHANNEL']['CCPD_TDC'] = 0
        self.dut['ENABLE_CHANNEL']['FE'] = 0
        self.dut['ENABLE_CHANNEL'].write()

        ## set gate for tlu communication
        self.dut["CCPD_GATE"].reset()
        if extrig=="tlu" or extrig=="inj":
            self.dut["CCPD_GATE"]["REPEAT"]=1
            self.dut["CCPD_GATE"]["DELAY"]=1
            self.dut["CCPD_GATE"]["WIDTH"]=adc_count+2
            self.dut["CCPD_GATE"]["EN"]=1
        else:
            self.dut["CCPD_GATE"]["EN"]=0
         
        self.dut["CCPD_INJ"].reset()
        if extrig=="inj":
            self.dut["CCPD_INJ"]["REPEAT"]=1
            self.dut["CCPD_INJ"]["DELAY"]=1
            self.dut["CCPD_INJ"]["WIDTH"]=adc_count+2
            self.dut["CCPD_INJ"]["EN"]=1
        else:
            self.dut["CCPD_INJ"]["EN"]=0
        
        if extrig=="tlu" or extrig=="inj":
            self.dut["CCPD_SW"]["ADC_TRIG_GATE"]=1
        else:
            self.dut["CCPD_SW"]["ADC_TRIG_GATE"]=0
        self.dut["CCPD_SW"].write()

        for i in range(4):
            ## set adc
            self.dut["fadc%d_rx"%i].reset()
            self.dut["fadc%d_rx"%i].set_data_count(adc_count)
            self.dut["fadc%d_rx"%i].set_delay(adc_delay)
            self.dut["fadc%d_rx"%i].set_single_data(True)
            if extrig=="tlu" or extrig=="inj" or extrig=="th":
                self.dut["fadc%d_rx"%i].set_en_trigger(True)
            else:
                self.dut["fadc%d_rx"%i].set_en_trigger(False)      
        s="set_adc: adc_count=%d adc_delay=%d extrig=%s"%(adc_count,adc_delay,extrig)
        self.logger.info(s)    
    def get_adc_data(self):
        if self.dut["fadc0_rx"].get_en_trigger()==0:
                self.dut['sram'].reset()
                for i in range(4):
                    self.dut['fadc%d_rx'%i].start()
                flg=0
        elif self.dut["CCPD_SW"]["ADC_TRIG_GATE"]==1: ## inj TODO exclude extrig of gate==0 (tlu)
                self.dut['sram'].reset()
                self.dut['CCPD_GATE'].start()
                flg=1
        else:
                flg=2  ## th TODO tlu
        nmdata = self.dut['sram'].get_data()
        while (flg==0 and self.dut['fadc0_rx'].is_done()==0) or (flg==1 and self.dut['fadc0_rx'].is_done()==0) \
              or (flg==2 and len(nmdata)!=0) :
            nmdata = np.append(nmdata, self.dut['sram'].get_data())
            time.sleep(0.001)
        return np.append(nmdata, self.dut['sram'].get_data())

    def analyse_adc(self,dat):
        ids=[0xe0000000,0xC0000000,0xA0000000,0x80000000]
        ret=[]
        for i in range(4):
            ret.append(dat[(dat & ids[i])==ids[i]])
            if ret[i][0] & 0x10000000 != 0x10000000:
                self.logger.info("analyse_adc: first adc data was not the first data 0x%x"%dat[0])
            ret[i]=ret[i] & 0x00003FFF
        return ret

    def configure(self,**karg):
        """ anabuf=col,inj=[pix],ampout=row,BLR=15,VNBias=1,VNFB=5,VNLogic=10,VPLoad=10,VNSF=30,VP=30,VPAB=20
        """
        for  k,v in karg.iteritems():
            col=[0]*300
            if "anabuf" in k:
                if isinstance(v,int):
                    col[v]=1
                elif isinstance(v,str):
                    pass # if v=="none" then all col==0
                for i,c in enumerate(col):
                    self.dut["CCPD_CONF_A"]["COL"][300-i-1]["CaliAnaBuf"]=c
                    self.dut["CCPD_CONF_A"]["COL"][300-i-1]["TestToAnaBuf"]=c
                    self.dut["CCPD_CONF_A"]["COL"][300-i-1]["InjEn"]=0
                for i in range(23):
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["InjEn"]=0
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["AmpToTest"]=0
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["Ampout"]=0    
            elif "inj" in k:
                col=[0]*300
                row=[0]*23
                if isinstance(v, str):
                   if v=="none":
                       v=[]
                elif isinstance(v[0],int):
                    v=[v]
                for vv in v:
                    row[vv[0]]=1
                    col[vv[1]]=1
                for i,r in enumerate(row):
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["InjEn"]=r
                for i,c in enumerate(col):
                    self.dut["CCPD_CONF_A"]["COL"][300-i-1]["InjEn"]=c
            elif "mon" in k:
                col=[0]*300
                row=[0]*23
                if isinstance(v, str):
                   if v=="none":
                       v=[]
                elif isinstance(v[0],int):
                    v=[v]
                for vv in v:
                    row[vv[0]]=1
                    col[vv[1]]=1
                for i,r in enumerate(row):
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["AmpToTest"]=r
                for i,c in enumerate(col):
                    self.dut["CCPD_CONF_A"]["COL"][300-i-1]["TestToAnaBuf"]=c
            elif "ampout" == k:
                row=[0]*23
                if isinstance(v, str):
                   if v=="none":
                       v=[]
                elif isinstance(v,int):
                    v=[v]
                for vv in v:
                    row[vv]=1
                for i,r in enumerate(row):
                    self.dut["CCPD_CONF_A"]["ROW"][23-i-1]["Ampout"]=r
            else:
                self.dut["CCPD_CONF_A"][k]=v
        #self.dut["CCPD_CONF_A"]["DACOut0"]=3
        self.dut["CCPD_CONF_A"]["EnCCPD"]=0
        self.dut["CCPD_CONF_A"].write()
        self.dut["CCPD_CONF_A"].start()
        while not self.dut['CCPD_CONF_A'].is_ready:
            pass
        print self.dut["CCPD_CONF_A"][:]
    def set_inj_all(self,inj_high=1,inj_low=0,inj_n=0,inj_width=500,extrig=False):
        self.dut["CCPD_INJ"].reset()
        if inj_n==1:
            self.dut["CCPD_INJ"]["DELAY"]=5
        else:
            self.dut["CCPD_INJ"]["DELAY"]=inj_width
        self.dut["CCPD_INJ"]["WIDTH"]=inj_width
        self.dut["CCPD_INJ"]["REPEAT"]=inj_n
        self.dut["CCPD_INJ"]["EN"]=True
        self.dut["CCPD_INJ_HIGH"].set_voltage(inj_high)
        self.inj_high=inj_high
        self.dut["CCPD_INJ_LOW"].set_voltage(inj_low)
        self.inj_low=inj_low
        
        self.dut["CCPD_GATE"].reset()
        self.dut["CCPD_GATE"]["REPEAT"]=1
        self.dut["CCPD_GATE"]["DELAY"]=1
        if inj_n==0 :
            self.dut["CCPD_GATE"]["WIDTH"]=(self.dut["CCPD_INJ"]["DELAY"]+inj_width)*100+5
        else:
            self.dut["CCPD_GATE"]["WIDTH"]=(self.dut["CCPD_INJ"]["DELAY"]+inj_width)*inj_n+5
        self.dut["CCPD_GATE"]["EN"]=extrig

    def inject(self):
        self.dut["CCPD_INJ"]['START']=1
        while not self.dut['CCPD_INJ']['READY'] and self.dut["CCPD_INJ"]["REPEAT"]!=0:
            time.sleep(0.003)
    def show(self):
        dat={}
        dat["CCPD_CONF_A"]=str(self.dut["CCPD_CONF_A"][:])
        dat["CCPD_INJ"]=self.dut["CCPD_INJ"].get_configuration()
        dat["CCPD_GATE"]=self.dut["CCPD_GATE"].get_configuration()
        dat["CCPD_INJ_VOLTAGE"]="%f,%f"%(self.inj_high, self.inj_low)
        self.logger.show(dat)

if __name__=="__main__":
    chip = H35DEMO()
    chip.init()
