# Calibration of heater PID settings
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import heaters

class PIDCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('PID_CALIBRATE', self.cmd_PID_CALIBRATE,
                               desc=self.cmd_PID_CALIBRATE_help)
    cmd_PID_CALIBRATE_help = "Run PID calibration test"
    def cmd_PID_CALIBRATE(self, gcmd):
        heater_name = gcmd.get('HEATER')
        target = gcmd.get_float('TARGET')
        gamma = gcmd.get_float('GAMMA')
        write_file = gcmd.get_int('WRITE_FILE', 0)
        pheaters = self.printer.lookup_object('heaters')
        try:
            heater = pheaters.lookup_heater(heater_name)
        except self.printer.config_error as e:
            raise gcmd.error(str(e))
        self.printer.lookup_object('toolhead').get_last_move_time()
        calibrate = ControlAutoTune(heater, target,gamma)
        old_control = heater.set_control(calibrate)
        try:
            pheaters.set_temperature(heater, target, True)
        except self.printer.command_error as e:
            heater.set_control(old_control)
            raise
        heater.set_control(old_control)
        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        if calibrate.check_busy(0., 0., 0.):
            raise gcmd.error("pid_calibrate interrupted")
        # Log and report results
        Kp, Ki, Kd = calibrate.calc_final_pid()
        logging.info("Autotune: final: Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd)
        gcmd.respond_info(
            "PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "The SAVE_CONFIG command will update the printer config file\n"
            "with these parameters and restart the printer." % (Kp, Ki, Kd))
        # Store results for SAVE_CONFIG
        configfile = self.printer.lookup_object('configfile')
        configfile.set(heater_name, 'control', 'pid')
        configfile.set(heater_name, 'pid_Kp', "%.3f" % (Kp,))
        configfile.set(heater_name, 'pid_Ki', "%.3f" % (Ki,))
        configfile.set(heater_name, 'pid_Kd', "%.3f" % (Kd,))

# ~~TUNE_PID_DELTA = 5.0

class ControlAutoTune:
    def __init__(self, heater, target, gamma):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.calibrate_temp = target
        # Heating control
        self.heating = False
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []
        self.temp_samples = []
        self.phase_temps = []
        self.phase_pwms = []
        self.phase = 0
        self.target_PWM = 1
        self.delt =0
        self.prev_delt = 0
        self.phase_start = 0
        self.h = 0          #hysterics value
        self.bands=[]       #stores temp band values are added in as bands[0] = upper and bands[1] = lower
        self.pwm_amps=[]    #stores current up and down amps for pwm
        self.sampling=[]    #stores temps for each halfcycle for checking amps
        self.halfcycles= [[],[]]  #halfcycles = [[[tdwn,pwm],[tdwn,pwm],...],[[tup,pwm],[tup,pwm],...]
        self.gamma = gamma
        
    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append(
                (read_time + self.heater.get_pwm_delay(), value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)
    def temperature_update(self, read_time, temp, target_temp):
        self.temp_samples.append((read_time, temp))
        if self.phase == 0:
            if not self.heating and temp <= target_temp:
                self.set_pwm(read_time, self.heater_max_power)
                self.heating = True
            if temp > target_temp:
                self.set_pwm(read_time, 0.)
                self.heating = False
                self.phase = 1
            # maybe add some sort of time out error here? Probably a good idea
            
        elif self.phase == 1:
            self.phase_temps.append(temp)
            self.set_pwm(read_time, min(self.heater_max_power, self.target_PWM))
            self.target_PWM = self.target_PWM + 1    #this growth may be too slow, but faster growth will lead to inacuracy
            
            if len(self.phase_temps) >= 4:
                 self.delt = self.phase_temps[-1] - self.phase_temps[-2]
                 self.prev_delt= self.phase_temps[-2] - self.phase_temps[-3]
                 if abs(abs(self.delt)-abs(self.prev_delt))/abs(self.prev_delt) <= 0.01:
                        self.phase = 2
                        self.phase_temps = []
                        self.phase_pwms = []
                        self.target_PWM = self.target_PWM - 1 
            
        elif self.phase == 2:
            if self.phase_start == 0:
                self.phase_start = read_time
            if read_time - self.phase_start > 10:
                self.phase_temps.sort(reverse= True)
                tmax=self.phase_temps[0]
                tmin=self.phase_temps[-1]
                self.h = 3*(tmax-tmin)/2
                self.bands.append([self.calibrate_temp+4*self.h,self.calibrate_temp+2+4*self.gamma*self.h])
                self.bands.append([self.calibrate_temp-4*self.h,self.calibrate_temp-2-4*self.gamma*self.h])
                self.bands.append([self.calibrate_temp+(2+4*self.gamma*self.h+4*self.gamma*self.h)/2,self.calibrate_temp-(4*self.h+(2+4*self.h))/2])
                self.phase = 3
                self.phase_temps = []
                self.phase_pwms = [self.target_PWM]
            else:
                self.phase_temps.append(temp)
            
            
        elif self.phase == 3:
            if temp in range(self.bands[0][0],self.bands[0][1]):
                self.pwm_amps.append(self.phase_pwms[-2])
                self.pwm_amps.append(self.phase_pwms[-2]/self.gamma)
                self.set_pwm(read_time, self.pwm_amps[1])
                self.phase = 4
                self.phase_temps = []
                self.phase_pwms = [[read_time, self.pwm_amps[1]]]
            else:
                self.phase_pwms.append(self.phase_pwms[-1]+1)
                self.set_pwm(read_time, min(self.heater_max_power, self.phase_pwms[-1]))
            
            
        elif self.phase == 4:
            if not self.heating and temp in range(self.bands[1][1],self.bands[1][0]): #if not heating and in lower band
                self.phase_temps.append((read_time, temp))
                for samples in self.phase_temps:
                    if samples[0] in range(self.phase_pwms[-1][0], read_time):
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[-1] > self.bands[0][1]:
                    self.pwm_amps[0]=self.pwm_amps[0]*(self.bands[2][0]/self.sampling[-1])
                self.sampling=[]
                self.heating= True
                self.set_pwm(read_time,self.pwm_amps[0])
                self.phase_pwms.append((read_time,self.pwm_amps[0]))
                self.halfcycles[1].append([self.phase_pwms[-2][0]-self.phase_pwms[-1][0],self.phase_pwms[-2][1]])
                if len(self.halfcycles[0]) and len(self.halfcycles[0]) >= 2:
                    if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                        self.set_pwm(read_time,0.)
                        self.phase = 5
                
            elif self.heating and temp in range(self.bands[0][0],self.bands[0][1]): #if heating and in upper band
                self.phase_temps.append((read_time, temp))
                for samples in self.phase_temps:
                    if samples[0] in range(self.phase_pwms[-1][0], read_time):
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[0] < self.bands[1][1]:
                    self.pwm_amps[1]=self.pwm_amps[1]*(self.bands[2][1]/self.sampling[0])
                self.sampling=[]
                self.heating= False
                self.set_pwm(read_time,self.pwm_amps[1])
                self.phase_pwms.append((read_time,self.pwm_amps[1]))
                self.halfcycles[0].append([self.phase_pwms[-2][0]-self.phase_pwms[-1][0],self.phase_pwms[-2][1]])
                if len(self.halfcycles[0]) and len(self.halfcycles[0]) >= 2:
                    if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                        self.set_pwm(read_time,0.)
                        self.phase = 5
                        
            else: #if not at a switching point
                self.phase_temps.append((read_time, temp))                
                    
        
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if self.heating or self.phase != 5:
            return True
        return False
    
    # Analysis
    def calc_final_pid(self):
        rho = max(max(self.halfcycles[0][-4:][0]),max(self.halfcycles[1][-4:][0]))/ min(min(self.halfcycles[0][-4:][0]),min(self.halfcycles[1][-4:][0]))
        tau= (self.gamma - rho)/((self.gamma-1)(0.35*rho+0.65))
        pwmInt=self.halfcycles[1][-1][0]*self.halfcycles[1][-1][1] - self.halfcycles[0][-1][0]*self.halfcycles[0][-1][1]
        tempInt=0
        cycleTemps=[]
        for samples in self.phase_temps:
            if samples[0] in range(self.phase_pwms[-3][0],self.phase_temps[-1][0]):
                cycleTemps.append(samples)
        for pairs in cycleTemps:
            if cycleTemps.index(pairs)+1 <= len(cycleTemps):
                tempInt += (cycleTemps[cycleTemps.index(pairs)][0] - pairs[0])*(cycleTemps[cycleTemps.index(pairs)][1] + pairs[1])/2
        Kp=tempInt/pwmInt
        T=self.halfcycles[1][-1][0]/math.log(((self.h/Kp) - self.halfcycles[1][-1][1] + math.exp(tau/(tau-1))*(self.halfcycles[1][-1][1]+self.halfcycles[0][-1][1]))/(self.halfcycles[0][-1][1]-(self.h/Kp)))
        L=T*(tau/(tau-1))
        K=(0.2*L+0.45*T)/(Kp*L)
        Ti=L*(0.4*L+0.8*T)/(L+0.1*T)
        Td=0.5*L*T/(0.3*L+T)
        Kp=K
        Ki=K/Ti
        Kd=K*Td
        return Kp, Ki, Kd
        
    # Offline analysis helper
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        f = open(filename, "wb")
        f.write('\n'.join(pwm + out))
        f.close()
        
        
def load_config(config):
    return PIDCalibrate(config)
