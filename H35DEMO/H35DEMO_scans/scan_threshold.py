import time
import logging
import numpy as np
import progressbar
from threading import Timer

from pybar.analysis.analyze_raw_data import AnalyzeRawData
from pybar.fei4.register_utils import invert_pixel_mask, make_box_pixel_mask_from_col_row
from pybar.fei4_run_base import Fei4RunBase
from pybar.run_manager import RunManager



class ThresholdScan(Fei4RunBase):
    '''Standard Threshold Scan

    Implementation of a standard threshold scan.
    '''
    _default_run_conf = {
        "trig_count": 0,  # FE-I4 trigger count, number of consecutive BCs, 0 means 16, from 0 to 15
        "trigger_latency": 235,  # FE-I4 trigger latency, in BCs, external scintillator / TLU / HitOR: 232, USBpix self-trigger: 220
        "trigger_delay": 8,  # trigger delay, in BCs
        "trigger_rate_limit": 0,  # artificially limiting the trigger rate, in BCs (25ns)
        "col_span": [1, 80],  # defining active column interval, 2-tuple, from 1 to 80
        "row_span": [1, 336],  # defining active row interval, 2-tuple, from 1 to 336
        "update_c_high_low": False,  # if True, use col_span and row_span to define an active region regardless of the Enable pixel register. If False, use col_span and row_span to define active region by also taking Enable pixel register into account.
        "use_enable_mask_for_imon": False,
        "no_data_timeout": 10,  # no data timeout after which the scan will be aborted, in seconds
        "scan_timeout": 10,  # timeout for scan after which the scan will be stopped, in seconds
        "max_triggers": 100000,  # maximum triggers after which the scan will be stopped, in seconds
        "enable_tdc": False,  # if True, enables TDC (use RX2)
        "reset_rx_on_error": False,  # long scans have a high propability for ESD related data transmission errors; recover and continue here
        "trig_src": "inj", # monhit, rj45, inj
        "tmp_t":0,
        "ccpd_inj": True,
        "scan_parameters": [('GDAC', [0, 300])],  # the PlsrDAC range
        "step_size": 5,  # step size of the PlsrDAC during scan
    }

    def configure(self):
        #print "============start configure============"
        self.dut['CMD']['EN_EXT_TRIGGER'] = False
        self.dut["ENABLE_CHANNEL"]["FE"]=0
        self.dut["ENABLE_CHANNEL"]["TLU"]=0
        self.dut["ENABLE_CHANNEL"].write()

        commands = []
        commands.extend(self.register.get_commands("ConfMode"))
        # Enable
        enable_pixel_mask = make_box_pixel_mask_from_col_row(column=self.col_span, row=self.row_span)
        enable_pixel_mask = np.logical_and(enable_pixel_mask, self.register.get_pixel_register_value('Enable'))
        self.register.set_pixel_register_value('Enable', enable_pixel_mask)
        commands.extend(self.register.get_commands("WrFrontEnd", same_mask_for_all_dc=False, name='Enable'))
        # Imon
        if self.use_enable_mask_for_imon:
            imon_pixel_mask = invert_pixel_mask(enable_pixel_mask)
        else:
            imon_pixel_mask = make_box_pixel_mask_from_col_row(column=self.col_span, row=self.row_span, default=1, value=0)  # 0 for selected columns, else 1
            imon_pixel_mask = np.logical_or(imon_pixel_mask, self.register.get_pixel_register_value('Imon'))
        self.register.set_pixel_register_value('Imon', imon_pixel_mask)
        commands.extend(self.register.get_commands("WrFrontEnd", same_mask_for_all_dc=False, name='Imon'))
        if self.update_c_high_low:
            # C_High
            self.register.set_pixel_register_value('C_High', 0)
            commands.extend(self.register.get_commands("WrFrontEnd", same_mask_for_all_dc=True, name='C_High'))
            # C_Low
            self.register.set_pixel_register_value('C_Low', 0)
            commands.extend(self.register.get_commands("WrFrontEnd", same_mask_for_all_dc=True, name='C_Low'))
        # Registers
        self.register.set_global_register_value("Trig_Lat", self.trigger_latency)  # set trigger latency
        self.register.set_global_register_value("Trig_Count", self.trig_count)  # set number of consecutive triggers
        commands.extend(self.register.get_commands("WrRegister", name=["Trig_Lat", "Trig_Count"]))
        commands.extend(self.register.get_commands("RunMode"))
        self.register_utils.send_commands(commands)

        #self.dut['CMD']['EN_EXT_TRIGGER'] = True
        #time.sleep(self.tmp_t)
        #self.dut['CMD']['EN_EXT_TRIGGER'] = False
        self.dut["ENABLE_CHANNEL"]["FE"]=1
        
        if self.trig_src=="rj45":
            self.dut["ENABLE_CHANNEL"]["MONHIT"]=0
            self.dut["ENABLE_CHANNEL"]["RJ45"]=1
            self.dut["ENABLE_CHANNEL"]["TLU"]=1
        elif self.trig_src=="monhit":
            self.dut["ENABLE_CHANNEL"]["MONHIT"]=1
            self.dut["ENABLE_CHANNEL"]["RJ45"]=0
            self.dut["ENABLE_CHANNEL"]["TLU"]=0
        else: ##'inj'
            self.dut["ENABLE_CHANNEL"]["MONHIT"]=0
            self.dut["ENABLE_CHANNEL"]["RJ45"]=0
            self.dut["ENABLE_CHANNEL"]["TLU"]=0
        self.dut["ENABLE_CHANNEL"].write()
        #print "============configure done============"

    def scan(self):
        scan_parameter_range = [0, (2 ** (self.register.global_registers['Vthin_AltCoarse']['bitlength']+self.register.global_registers['Vthin_AltFine']['bitlength']))]
        if self.scan_parameters.GDAC[0]:
            scan_parameter_range[0] = self.scan_parameters.GDAC[0]
        if self.scan_parameters.GDAC[1]:
            scan_parameter_range[1] = self.scan_parameters.GDAC[1]
        scan_parameter_range = range(scan_parameter_range[0], scan_parameter_range[1] + 1, self.step_size)
        logging.info("Scanning %s from %d to %d", 'GDAC', scan_parameter_range[0], scan_parameter_range[-1])
        t0=time.time()	
        for scan_parameter_value in scan_parameter_range:
            commands = []
            commands.extend(self.register.get_commands("ConfMode"))
            self.register.set_global_register_value('Vthin_AltFine', scan_parameter_value & 0xFF)
            self.register.set_global_register_value('Vthin_AltCoarse', (scan_parameter_value>>8) & 0xFF)
            commands.extend(self.register.get_commands("WrRegister", name=['Vthin_AltFine','Vthin_AltCoarse']))
            self.register_utils.send_commands(commands)
            lvl1_command = self.register.get_commands("zeros", length=self.trigger_delay)[0] \
                         + self.register.get_commands("LV1")[0] \
                         + self.register.get_commands("zeros", length=self.trigger_rate_limit)[0]
            self.register_utils.set_command(lvl1_command)
            self.dut['CMD']['EN_EXT_TRIGGER'] = True
                
            with self.readout(GDAC=scan_parameter_value):
                print scan_parameter_value, time.time()-t0, "current",self.dut["Vin"].get_current(unit="mA")
                self.dut["CCPD_PULSE_INJ"].start()
    def analyze(self):
        with AnalyzeRawData(raw_data_file=self.output_filename, create_pdf=True) as analyze_raw_data:
            analyze_raw_data.create_tot_hist = False
            #analyze_raw_data.create_fitted_threshold_hists = True
            analyze_raw_data.create_threshold_mask = True
            analyze_raw_data.n_injections = 100
            analyze_raw_data.interpreter.set_warning_output(False)  # so far the data structure in a threshold scan was always bad, too many warnings given
            analyze_raw_data.interpret_word_table()
            analyze_raw_data.interpreter.print_summary()
            analyze_raw_data.plot_histograms()

if __name__ == "__main__":
    RunManager('../configuration.yaml').run_run(ThresholdScan)
