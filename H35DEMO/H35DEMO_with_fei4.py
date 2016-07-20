import time, string, os ,sys

import logging
import numpy as np
np.set_printoptions(linewidth="nan", threshold="nan")
import matplotlib.pyplot as plt
import bitarray
import tables

import H35DEMO
import H35DEMO_scans
import pybar

class H35DEMOFei4(H35DEMO.H35DEMO):
    def init(self):
        # init fei4
        conf='/home/user/workspace/pyBAR/pyBAR_H35DEMO/pybar/configuration.yaml'
        self.rmg=pybar.run_manager.RunManager(conf)
        self.rmg.run_run(H35DEMO_scans.InitScan)
        self.dut=self.rmg.conf.dut
        # init H35DEMO
        self.dut['CCPD_CONF_A'].reset()
        self.dut['CCPD_CONF_A'].set_en(False)
        self.dut['CCPD_CONF_A'].set_size(1491)
        self.configure(BLR=15,VNBias=1,VNFB=5,VNLogic=10,VPLoad=10,VNSF=30,VP=30,VPAB=20)
        self.set_inj_all(inj_high=1,inj_low=0,inj_n=1,inj_width=500)
        self.logger.info("power_fei4: %.3fV(%.3fmA)"%(self.dut['Vin'].get_voltage(),self.dut['Vin'].get_current(unit="mA")))
        


    def test_fei4(self):
        self.dut["rx"]["FE"]=1
        self.dut["rx"].write()
        self.rmg.run_run(H35DEMO_scans.DigitalScan)
        self.rmg.run_run(H35DEMO_scans.AnalogScan)

    def tune_fei4(self,run_conf=None,tune="fei4"):
        self.dut["rx"]["FE"]=1
        self.dut["rx"].write()
        if tune=="fei4":
            self.rmg.run_run(ccpdlf_scans.Fei4Tuning,run_conf=run_conf)
        else:
            self.rmg.run_run(ccpdlf_scans.ThresholdBaselineTuning,run_conf=run_conf)

    def run_selftrigger(self,run_conf=None,use_thread=False):
       self.rmg.run_run(ccpdlf_scans.FEI4SelfTriggerScan,run_conf=run_conf,use_thread=use_thread)
    def run_exttrigger(self,run_conf=None,use_thread=False):
       if run_conf==None:
           run_conf={"row_offset":self.row_offset}
       else:
           if not "row_offset" in run_conf.keys():
               run_conf["row_offset"]=self.row_offset
       self.rmg.run_run(ccpdlf_scans.ExtTriggerScan,run_conf=run_conf,use_thread=use_thread)
    def set_hitmon_en(self,pix):
        imon_pixel_mask=self.rmg.current_run.register.get_pixel_register_value('Imon')
        if isinstance(pix,str):
            if pix=="all":
                imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=0
            else: ###"none"
                imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=1
        else:
            if isinstance(pix[0], int):
                pix=[pix]
            if np.shape(pix)[1]==2:
              imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=1
              for p in pix:
                fe_p=self.ccpdlf2fei4(p,True)
                imon_pixel_mask[fe_p[0],fe_p[1]]=0
                ###
                pix3=[]
                pix3.append(self.fei42ccpdlf(fe_p,0))
                pix3.append(self.fei42ccpdlf(fe_p,1))
                pix3.append(self.fei42ccpdlf(fe_p,2))
                self.logger.info("set_hitmon_en fei4:[%d,%d],pix:%s"%(fe_p[0],fe_p[1],str(pix3)))
            elif np.shape(pix)[1]==76:
                imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=pix
        self.rmg.current_run.register.set_pixel_register_value('Imon', imon_pixel_mask)
        commands = []
        commands.extend(self.rmg.current_run.register.get_commands("ConfMode"))
        commands.extend(self.rmg.current_run.register.get_commands("WrFrontEnd", same_mask_for_all_dc=False, name='Imon'))
        commands.extend(self.rmg.current_run.register.get_commands("RunMode"))
        self.rmg.current_run.register_utils.send_commands(commands)
    def set_fdac_pix(self,pix,fdac):
        fdac_pixel_mask=self.rmg.current_run.register.get_pixel_register_value('FDAC')
        if isinstance(pix,str):
            if pix=="all":
                imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=fdac
        else:
            if isinstance(pix[0], int):
                pix=[pix]
            for p in pix:
                fe_p=self.ccpdlf2fei4(p,True)
                fdac_pixel_mask[fe_p[0],fe_p[1]]=fdac
                ###
                pix3=[]
                pix3.append(self.fei42ccpdlf(fe_p,0))
                pix3.append(self.fei42ccpdlf(fe_p,1))
                pix3.append(self.fei42ccpdlf(fe_p,2))
                self.logger.info("set_hitmon_en fdac:%d fei4:[%d,%d],pix:"%(fdac, fe_p[0],fe_p[1]),pix3)
        self.rmg.current_run.register.set_pixel_register_value('FDAC', fdac_pixel_mask)
        commands = []
        commands.extend(self.rmg.current_run.register.get_commands("ConfMode"))
        commands.extend(self.rmg.current_run.register.get_commands("WrFrontEnd", same_mask_for_all_dc=False, name=["FDAC"]))
        commands.extend(self.rmg.current_run.register.get_commands("RunMode"))
        self.rmg.current_run.register_utils.send_commands(commands)
    def tune_fdac(self,wgt2=16):
        self.set_preamp_en("all")
        #self.set_inj_all(inj_high=0.3,inj_n=100)
        self.set_global(WGT0=0,WGT1=0,WGT2=wgt2,VSTRETCH=1,LSBdacL=63)
        #self.set_th(0.765)
        for j in range(24):
          pix=[]
          for i in range(2,114,3):
            pix.append([j,i])
          self.set_hitmon_en(pix)
          self.set_inj_en(pix)
          dat=np.empty([len(pix),16])
          for fdac in range(0,16):
            self.set_fdac_pix(pix,fdac)
            self.run_exttrigger({"trig_src":"inj",'ccpd_inj':True,'scan_timeout':1})
            with tables.open_file(self.get_fei4file()) as tb:
                 for i,p in enumerate(pix):
                     fep0,fep1=self.ccpdlf2fei4(p,True)
                     d=tb.root.HistTotPixel[fep1,fep0]
                     mean=np.sum(d * x) /float(np.sum(d))
                     dat[i,fdac]=mean
          with open("%d-wgt2=%d.npy"%(j,wgt2),"wb") as f:
            np.save(f,pix)
            np.save(f,dat)
            arg=np.empty(np.shape(dat)[0])
            for i in range(np.shape(dat)[0]):
                arg[i]=ccpdTools.find_arg(dat[i,:],8)
                if int(arg[i])>16:
                   print i,arg[i],dat[i,:]
                   arg[i]=15
                elif int(arg[i])<0:
                    print i,arg[i],dat[i,:]
                    arg[i]=0
                self.set_fdac_pix(pix[i],int(arg[i]))
            np.save(f,arg)
    def scan_gdac(self,pix,b,e,s,th=0.8,inj_high=1.5):
        self.set_preamp_en(pix)
        self.set_inj_en(pix)
        self.set_mon_en(pix)
        self.set_th(th)
        if pix[0]%3==0:
            self.set_global(WGT0=63,WGT1=0,WGT2=0)
        elif pix[0]%3==1:
            self.set_global(WGT0=0,WGT1=63,WGT2=0)
        elif pix[0]%3==2:
            self.set_global(WGT0=0,WGT1=0,WGT2=63)
        self.set_inj_all(inj_n=100,inj_high=inj_high)
        for g in range(38,110,1):
            self.run_exttrigger(run_conf={"gdac":g})
            try:
               dat=ccpdlf_util.load_fei4data(ccpdlf_util.get_fei4file(),row_offset=self.row_offset)
               feix=ccpdlf_util.ccpdlf2fei4(pix,ab="False")
               s="fepix=[%d,%d] cnt=%d"%(fepix[0],fepix[1],dat[fepix[0],fepix[1]])
               if dat[fepix[0],fepix[1]]==0:
                    break
            except:
               s="no data"
            c.logger("run_exttrigger() %d %d %d %s"%(i,j,s))

    def tune_tdac_fei4(self,th=0.77,LSBdacL=63):
        th=0.78
        LSBdacL=63
        wgt0=6
        wgt1=10
        wgt2=13
        c.set_inj_all(inj_high=0.3,inj_n=100)
        c.set_th(th)
        pix=np.load("pix.npy")
        c.set_preamp_en(pix)
        tdac=np.copy(c.tdac)
        cnt=np.empty([24,114,16])
        imon_pixel_mask=c.rmg.current_run.register.get_pixel_register_value('Imon')
        imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=0
        c.rmg.current_run.register.set_pixel_register_value('Imon', imon_pixel_mask)
        for wgt in range(3):
          if wgt==0:
            c.set_global(WGT0=wgt0,WGT1=0,WGT2=0,VSTRETCH=1,LSBdacL=LSBdacL)
          elif wgt==1:
            c.set_global(WGT0=0,WGT1=wgt1,WGT2=0,VSTRETCH=1,LSBdacL=LSBdacL)
          else:
            c.set_global(WGT0=0,WGT1=0,WGT2=wgt2,VSTRETCH=1,LSBdacL=LSBdacL)
          for i in range(0,24,1):
            flg=np.zeros(114/3)
            for t in range(15,-1,-1):
              pix=[]
              for j in range(len(flg)-1):
                  if flg[j]==0:
                     tdac[i,j*3+wgt]=t
                     pix.append([i,j*3+wgt])
              if len(pix)==0:
                  print flg
                  break
              c.set_inj_en(pix)
              c.set_tdac(tdac)
              c.set_mon_en(pix[0])
              c.run_exttrigger({'scan_timeout':1,"trig_src":"inj","ccpd_inj":True})
              dat=ccpdlf_util.load_fei4data(ccpdlf_util.get_fei4file(),row_offset=self.row_offset)
              for p in pix:
                  p0,p1=ccpdlf2fei4(p)
                  #print tdac[pix[j][0],pix[j][1]],flg[j],"%d-%d"%(p0,p1),dat[p0,p1]
                  if dat[p0,p1]>5:
                      flg[p[1]/3]=1
                      tdac[p[0],p[1]]=min(tdac[p[0],p[1]]+1,15)
                  print "%2d,%d,%2d-%3d"%(tdac[p[0],p[1]],flg[p[1]/3],p0,p1),dat[p0,p1]
                  cnt[p0,p1,t]=dat[p0,p1]
              if np.all(flg==1):
                  break

    def get_tot_img(self):
            wgt2=16
            wgt1=7
            wgt0=4
            tot2=8
            tot1=4
            tot0=2
            x=np.arange(16)
            dat=np.empty([12,76,16,7])
            c.set_preamp_en("all")
            c.set_inj_all(inj_high=0.5,inj_n=100)
            c.set_th(0.765)
            for k_i,k in enumerate([[0],[1],[2],[0,1],[1,2],[2,0],[0,1,2]]):
              c.set_global(WGT0=wgt0,WGT1=wgt1,WGT2=wgt2,VSTRETCH=1,LSBdacL=63) 
              for j in range(24):
                pix=[]
                for i in range(0,114,3):
                    fep=c.ccpdlf2fei4([j,i],False)
                    for kk in range(3):
                        if kk in k:
                            pix.append(c.fei42ccpdlf(fep,kk,False))
                c.set_hitmon_en(pix)
                c.set_inj_en(pix)
                c.run_exttrigger({"trig_src":"inj",'ccpd_inj':True,'scan_timeout':1})
                with tables.open_file(get_fei4file()) as tb:
                  for i,p in enumerate(pix):
                     fep0,fep1=c.ccpdlf2fei4(p,True)
                     #print i,p,fep0,fep1,
                     dat[fep0,fep1-self.row_offset,:,k_i]=tb.root.HistTotPixel[fep1,fep0]
            np.save("tot.npy",dat)
            #plt.hist(arg,bins=np.arange(16),histtype="step");

    def tune_tdac_fei4noise(self,th=0.77,LSBdacL=63):
        th=0.752
        LSBdacL=63
        c.set_tdac(15)
        c.set_th(th)
        pix=np.load("pix.npy")
        c.set_preamp_en("all")
        c.set_mon_en('none')
        c.set_inj_en("none")
        tdac=np.copy(c.tdac)
        c.set_global(WGT0=0,WGT1=0,WGT2=63,VSTRETCH=1,LSBdacL=LSBdacL)
        cnt=np.empty([12,76,16])
        flg=np.zeros([24,114])
        c.set_inj_all(inj_n=1,inj_high=0.2)
        for k in range(2):
          imon_pixel_mask=c.rmg.current_run.register.get_pixel_register_value('Imon')
          imon_pixel_mask[:12,self.row_offset:self.row_offset+76]=0
          c.rmg.current_run.register.set_pixel_register_value('Imon', imon_pixel_mask)
          if k==2:
              c.set_global(WGT0=0,WGT1=0,WGT2=63,VSTRETCH=1,LSBdacL=LSBdacL)
          if k==1:
              c.set_global(WGT0=0,WGT1=63,WGT2=0,VSTRETCH=1,LSBdacL=LSBdacL)
          else:
              c.set_global(WGT0=63,WGT1=0,WGT2=0,VSTRETCH=1,LSBdacL=LSBdacL)
          for t in range(15,-1,-1):
            tdac[flg==0]=t
            c.set_tdac(tdac)
            while True:
              c.run_exttrigger({'scan_timeout':1,"trig_src":"monhit","ccpd_inj":False})
              cnt[:,:,t]=ccpdlf_util.load_fei4data(ccpdlf_util.get_fei4file(),row_offset=self.row_offset)
              arg=np.argwhere(cnt[:,:,t]>5)
              if len(arg)==0:
                break
              for a in arg:
                  p0,p1=fei42ccpdlf(a,k)
                  #if imon_pixel_mask[a[0],a[1]+self.row_offset]==1:
                   #    print "ERROR"
                    #   sys.exit()
                  imon_pixel_mask[a[0],a[1]+self.row_offset]=1
                  tdac[p0,p1]=min(tdac[p0,p1]+1,15)
                  print tdac[p0,p1],flg[p0,p1],"%d-%d"%(p0,p1),cnt[a[0],a[1],t]
                  flg[p0,p1]=1
              c.rmg.current_run.register.set_pixel_register_value('Imon', imon_pixel_mask)
              if np.all(imon_pixel_mask[:12,self.row_offset:self.row_offset+76]==1) or len(arg)!=0:
                  break


