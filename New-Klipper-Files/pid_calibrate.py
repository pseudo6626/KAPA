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
        self.heater_max_temp=heater.max_temp
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
        self.target_PWM = 0.05
	self.vary=0.0
        self.prev_delt = 0
        self.phase_start = 0
        self.h = 0          #hysterics value
        self.bands=[]       #stores temp band values are added in as bands[0] = upper and bands[1] = lower
        self.pwm_amps=[]    #stores current up and down amps for pwm
        self.sampling=[]    #stores temps for each halfcycle for checking amps
        self.halfcycles= [[],[]]  #halfcycles = [[[tdwn,pwm],[tdwn,pwm],...],[[tup,pwm],[tup,pwm],...]
        self.index=0;
        self.gamma = gamma
        self.mu=3	
        
    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append(
                (read_time + self.heater.get_pwm_delay(), value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)
    def temperature_update(self, read_time, temp, target_temp):
        self.temp_samples.append((read_time, temp))
        logging.info("phase: %f", self.phase)

        if self.phase == 0:
            if  temp <= target_temp:
                self.set_pwm(read_time, self.heater_max_power)
                self.heating = True
            if temp > target_temp:
                self.set_pwm(read_time, 0.)
                self.heating = False
                self.phase = 1
            # maybe add some sort of time out error here? Probably a good idea
            
        elif self.phase == 1:
            self.phase_temps.append(temp)
            logging.info(self.target_PWM)
            logging.info("temp: %f time: %f",temp,read_time)
            if len(self.phase_temps) > 20:
		self.vary=self.variance(self.phase_temps[-15:])
		logging.info(self.vary)
		if self.vary<=0.00025 and self.target_PWM >0.01: 
            		logging.info(self.vary)  
            		logging.info(self.phase_temps[-15:])                  
			self.phase = 2
                      	self.phase_temps = []
                      	self.phase_pwms = []
                      	self.calibrate_temp=temp
		elif self.phase_temps[-1] - self.phase_temps[-4] > 0.0:
                        self.target_PWM = max(self.target_PWM-self.vary,0.0)
                        self.set_pwm(read_time, self.target_PWM)
		elif self.phase_temps[-1] - self.phase_temps[-4] < 0.0:
                        self.target_PWM = min(self.target_PWM+self.vary,self.heater_max_power)
                        self.set_pwm(read_time, self.target_PWM)
            
        elif self.phase == 2:
            self.set_pwm(read_time,self.target_PWM)
            if self.phase_start == 0:
                self.phase_start = read_time
                self.heater.alter_target(self.calibrate_temp)
            if read_time - self.phase_start > 5:
                self.phase_temps.sort(reverse= True)
                tmax=self.phase_temps[0]
                tmin=self.phase_temps[-1]
                self.h = 3*(tmax-tmin)/2
                if self.h < 1:
                    if self.h <0.1:
                        self.mu=10
                    else:
                        self.mu=5
                if self.calibrate_temp+self.mu*self.gamma*self.h > self.heater_max_temp:
                    self.gamma = math.floor((self.heater_max_temp-self.calibrate_temp-5)/(self.h*self.mu))
                self.bands.append([self.calibrate_temp+self.mu*self.h,self.calibrate_temp+self.mu*self.gamma*self.h])
                self.bands.append([self.calibrate_temp-self.mu*self.h,self.calibrate_temp-self.mu*self.gamma*self.h])
                self.bands.append([self.calibrate_temp+self.mu*self.h*self.gamma,self.calibrate_temp-self.mu*self.h])
                self.phase = 4
                self.pwm_amps.append(min(self.target_PWM*self.gamma,self.heater_max_power))
                self.pwm_amps.append(0.0)
                logging.info(self.bands)
                logging.info(self.h)
                logging.info(self.target_PWM)
                self.phase_temps = []
                self.phase_pwms = [[read_time, self.pwm_amps[1]]]
                self.heater.alter_target(self.bands[1][1])
            else:
                self.phase_temps.append(temp)
            
            
        elif self.phase == 3:
            if int(temp) in range(self.bands[0][0],self.bands[0][1]): 
                self.pwm_amps.append(self.phase_pwms[-1])
                if self.phase_pwms[-1]/self.gamma -self.target_PWM > 0 and  self.phase_pwms[-1]/self.gamma -self.target_PWM < self.target_PWM:
                    self.pwm_amps.append(self.phase_pwms[-1]/self.gamma -self.target_PWM)
                else:
                    self.pwm_amps.append(0.0)
                self.set_pwm(read_time, self.pwm_amps[1])
                self.heater.alter_target(self.bands[2][1])
                self.phase = 4
                self.phase_temps = []
                self.phase_pwms = [[read_time, self.pwm_amps[1]]]
            else:
                self.heater.alter_target(self.bands[2][0])
                self.phase_pwms.append(min(self.phase_pwms[-1]+0.005,self.heater_max_power))
                self.set_pwm(read_time, min(self.heater_max_power, self.phase_pwms[-1]))
            
            
        elif self.phase == 4:
            if not self.heating and temp <= self.bands[1][0]: #if not heating and in lower band
                self.phase_temps.append((read_time, temp))
                for samples in self.phase_temps:
                    if samples[0] >= self.phase_pwms[-1][0] and samples[0] <= read_time:
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[-1] > self.bands[2][0]:
                    self.bands[0][1]=2*self.bands[2][0]-self.sampling[-1]
                if self.sampling[0] > self.bands[2][1]:
                    self.bands[0][1]=self.bands[0][1]+(self.bands[2][1]-self.sampling[-1])/2
                self.sampling=[]
                self.heating= True
                self.set_pwm(read_time,self.pwm_amps[0])
                self.heater.alter_target(self.bands[0][1])
                self.phase_pwms.append((read_time,self.pwm_amps[0]))
                self.halfcycles[1].append([self.phase_pwms[-1][0]-self.phase_pwms[-2][0],self.phase_pwms[-2][1]])
                logging.info("up time: %f",self.halfcycles[-1])
                if len(self.halfcycles[0]) and len(self.halfcycles[1]) >= 2:
                    logging.info("cycle dev: %f", abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))
                    #if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                    if abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0] and abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0] <=0.01:
                        self.phase = 5
                
            elif self.heating and temp >= self.bands[0][1]: #if heating and in upper band
                self.phase_temps.append((read_time, temp))
                for samples in self.phase_temps:
                    if samples[0] >= self.phase_pwms[-1][0] and samples[0] <= read_time:
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[0] < self.bands[2][1]:
                    self.bands[1][0]=2*self.bands[2][1]-self.sampling[0]
                if self.sampling[0] > self.bands[2][1]:
                    self.bands[1][0]=self.bands[1][0]-(self.sampling[0]-self.bands[2][1])/2
                self.sampling=[]
                self.heating= False
                self.set_pwm(read_time,self.pwm_amps[1])
                self.heater.alter_target(self.bands[1][0])
                self.phase_pwms.append((read_time,self.pwm_amps[1]))
                self.halfcycles[0].append([self.phase_pwms[-1][0]-self.phase_pwms[-2][0],self.phase_pwms[-2][1]])
                logging.info("down time: %f",self.halfcycles[-1])
                if len(self.halfcycles[0]) and len(self.halfcycles[1]) >= 2:
                    logging.info("cycle dev: %f", abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))
                    logging.info("up diff: %f down diff: %f",abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0],abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0])
                    #if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                    if abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0] and abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0] <=0.01:
                        self.phase = 5
                        
            else: #if not at a switching point
                self.phase_temps.append((read_time, temp))
                if self.heating:
                    self.set_pwm(read_time,self.pwm_amps[0])
		else:
                    self.set_pwm(read_time,self.pwm_amps[1])

        elif self.phase == 5:
            self.set_pwm(read_time,0.)
            self.heating= False
 
       
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if self.heating or self.phase != 5:
            return True
        return False
    
    def variance(self, data):
        # Number of observations
        n = len(data)
        # Mean of the data
        mean = sum(data) / n
        # Square deviations
        deviations = [(x - mean) ** 2 for x in data]
        # Variance
        variance = sum(deviations) / n
        return variance
    # Analysis
    def calc_final_pid(self):
        logging.info(self.halfcycles)
        uptimes=[]
        for vals in self.halfcycles[0][-4:]:
            uptimes.append(vals[0])
        minup=min(uptimes)
	maxup=max(uptimes)
        downtimes=[]
        for val in self.halfcycles[1][-4:]:
            downtimes.append(val[0])
        mindown=min(downtimes)
	maxdown=max(downtimes)
        logging.info("max up: %f min up: %f max down:%f min down: %f", maxup,minup,maxdown,mindown)
        rho = max(halfcycles[0][-1][0],halfcycles[1][-1][0])/ min(halfcycles[0][-1][0],halfcycles[1][-1][0])
        tau= (self.gamma - rho)/((self.gamma-1)*(0.35*rho+0.65))
        pwmInt=self.halfcycles[0][-1][0]*(self.halfcycles[0][-1][1]-self.target_PWM)*255 + self.halfcycles[1][-1][0]*(self.halfcycles[1][-1][1]-self.target_PWM)*255
        tempInt=0
        cycleTemps=[]
        for samples in self.phase_temps:
            if samples[0] >= self.phase_temps[-1][0]-(self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]):
                cycleTemps.append(samples)
        for pairs in cycleTemps:
            if cycleTemps.index(pairs)+1 < len(cycleTemps):
                tempInt += (cycleTemps[cycleTemps.index(pairs)+1][0] - pairs[0])*(((cycleTemps[cycleTemps.index(pairs)+1][1] + pairs[1])/2)-self.calibrate_temp)
        Kp=abs(tempInt/pwmInt)
        logging.info("rho: %f tau: %f pwmInt: %f tempInt: %f Kp: %f",rho,tau,pwmInt,tempInt,Kp)
        logging.info(cycleTemps)
        logging.info(self.phase_temps)
        logging.info(self.phase_pwms)
        T=self.halfcycles[0][-1][0]/math.log(((self.h/Kp) - self.target_PWM*255 + math.exp(tau/(1-tau))*(self.target_PWM*255+self.halfcycles[0][-1][1]*255))/(self.halfcycles[0][-1][1]*255-(self.h/Kp)))
        L=T*(tau/(1-tau))
        K=(0.2*L+0.45*T)/(Kp*L)
        Ti=L*(0.4*L+0.8*T)/(L+0.1*T)
        Td=0.5*L*T/(0.3*L+T)
        logging.info("T: %f L: %f K: %f Ti: %f Td: %f",T,L,K,Ti,Td)
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
