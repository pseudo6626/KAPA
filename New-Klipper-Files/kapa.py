# Calibration of heater PID settings
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import heaters

class KAPAPIDCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.preferences=[["max_testing_temp",-1],["max_threshold",0.0025]]
        for i in self.preferences:
             if config.get(i[0],None) != None:
                i[1]=config.getfloat(i[0])
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('KAPA', self.cmd_KAPA,
                               desc=self.cmd_KAPA_help)
    cmd_KAPA_help = "Run PID calibration test"
    def cmd_KAPA(self, gcmd):
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
        calibrate = ControlAutoTune(heater, target,gamma,self.printer.lookup_object('gcode'),self.preferences)
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
    def __init__(self, heater, target, gamma, gcode, pref):
        self.heater = heater
        self.gcode = gcode
        if pref[0][1] > 0:
            self.heater_max_temp=pref[0][1]
        else:
            self.heater_max_temp=heater.max_temp
        self.heater_max_power = heater.get_max_power()
        self.calibrate_temp = target
        # Heating control
        self.heating = False
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []   #global pwm samples recorded in (time,pwm). Does not reset between phases
        self.temp_samples = []  #global temp samples recorded in (time,temp). Does not reset between phases
        self.phase_temps = []   # the temp values and times for the current phase recorded in (time,temp). Reset between phases
        self.phase_pwms = []    #the pwm values and times for the current phase recorded in (time,pwm). Reset between phases
        self.phase = 0          #index used for moving through different phases of this program
        self.target_PWM = 0.05  #the pwm that is needed to just maintain the target temp
	self.vary=0.0        #used for measuring the variance in temps for phase 1
        self.phase_start = 0 #used for timer in phase 2
        self.h = 0          #hysterics value
        self.bands=[]       #stores temp band values are added in as bands[0] = upper , bands[1] = lower , bands[2] = ideal critical temps
        self.pwm_amps=[]    #stores up and down amps for pwm
        self.sampling=[]    #stores temps for each halfcycle for checking amps
        self.halfcycles= [[],[]]  #halfcycles = [[[tdwn,pwm],[tdwn,pwm],...],[[tup,pwm],[tup,pwm],...]
        self.gamma = gamma  #gamma sets the ratio between up and down amplitudes
        self.mu=3           #mu aids in defining how much greater than the hysterics values the target temps are
        self.mass=1.0       #a measure of the thermal inertia of the heater, identified as mass=Max_Power/(avg slope of initial heating curve). Used to dampen steady state process
        self.driver=0       #an estimation of the enviromental driving force cooling the heater
        self.threshold=0.000625   #the variance threshold for 5 seconds of temperatures to qualify steadstate pwm 
        self.max_threshold = pref[1][1]
        self.phase_call=0
        self.tau=-1
        self.rho=-1


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
        logging.info("max temp %f ", self.heater_max_temp)
        logging.info("max thresh %f ", self.max_threshold)



        if self.phase == 0:          #in this phase, the heater engages at full power till the target temp is reached
            if self.phase_call == 0:
                self.gcode.respond_info("phase: %d" % (self.phase))
                self.phase_call=1
            self.phase_temps.append([read_time,temp])
            if  temp <= target_temp:
                self.set_pwm(read_time, self.heater_max_power)
                self.heating = True
            if temp > target_temp:
                self.set_pwm(read_time, 0.)
                self.heating = False
                self.phase = 1
                self.phase_call=0
                self.mass=self.heater_max_power/((self.phase_temps[-1][1]-self.phase_temps[0][1])/(self.phase_temps[-1][0]-self.phase_temps[0][0]))
                self.gcode.respond_info("mass: %f" % (self.mass))
                logging.info("delt: %f delv: %f",((self.phase_temps[-1][1]-self.phase_temps[0][1]),(self.phase_temps[-1][0]-self.phase_temps[0][0])))
                self.phase_temps=[]          
	  # maybe add some sort of time out error here? Probably a good idea
            
        elif self.phase == 1:         #in this phase, the system seeks to identify what pwm value is need to maintain a constant temp with variance <= 0.00025c^2 per 0.33s
            if self.phase_call == 0:
                self.gcode.respond_info("phase: %d" % (self.phase))
                self.phase_call=1
            self.phase_temps.append([read_time,temp])
            if len(self.phase_temps) > 60 and self.phase_temps[-1][1] < self.phase_temps[-15][1]:
 		if self.driver == 0:
            		vals=[x[1] for x in self.phase_temps]
            		peak= vals.index(max(vals))
            		self.driver=self.mass*abs(self.phase_temps[-1][1]-self.phase_temps[peak][1])/(self.phase_temps[-1][0]-self.phase_temps[peak][0])
            		logging.info("driver: %f",self.driver)
            		self.target_PWM=min(self.driver,0.6)
            		self.phase_start=read_time

            if self.driver > 0:
		self.vary=self.variance([y[1] for y in self.phase_temps[-12:]])
		self.gcode.respond_info("variance: %f"% (self.vary))
		logging.info("current PWM: %f",self.target_PWM)
		if self.vary<=self.threshold and self.target_PWM >0.01 and read_time - self.phase_start>20: 
            		self.gcode.respond_info("variance: %f" % (self.vary))
			logging.info("steadystate pwm: %f" % (self.target_PWM))
			self.phase = 2
			self.phase_start=0.0
                      	self.phase_call=0
                      	self.phase_temps = []
                      	self.phase_pwms = []
                      	self.set_pwm(read_time,0.0)
                      	self.calibrate_temp=temp
		elif self.phase_temps[-1][1] - self.phase_temps[-4][1] > 0.0:
                        self.target_PWM = max(self.target_PWM-min(self.vary,0.003),0)
                        self.set_pwm(read_time, self.target_PWM)
		elif self.phase_temps[-1][1] - self.phase_temps[-4][1] < 0.0:
                        self.target_PWM = min(self.target_PWM+min(self.vary,0.003),self.heater_max_power)
                        self.set_pwm(read_time, self.target_PWM)
		self.threshold=min(0.000625+0.00003125*math.floor((len(self.phase_temps)-40)/15),self.max_threshold)
		self.gcode.respond_info("current threshold: %f" % (self.threshold))
            else:
		self.set_pwm(read_time, 0.0)


            
            
        elif self.phase == 2:         #in this phase, the system observes the temp for 5 seconds while the steady state pwm is applied. From this, it determines the hysterics value h, the target temperature bands, and the up and down pwm amplitudes
            if self.phase_call == 0:
                self.gcode.respond_info("phase: %d" % (self.phase))
                self.phase_call=1
            self.set_pwm(read_time,self.target_PWM)
            if self.phase_start == 0:
                self.phase_start = read_time
                self.heater.alter_target(self.calibrate_temp)
            if read_time - self.phase_start > 10:
                self.phase_temps.sort(reverse= True)
                tmax=self.phase_temps[0]
                tmin=self.phase_temps[-1]
                self.h = 3*(tmax-tmin)/2
               # if self.h < 1:
                   # if self.h <0.1:
                      #  self.mu=10
                    #else:
                       # self.mu=5
                if self.calibrate_temp+self.mu*self.gamma*self.h > self.heater_max_temp:
                    self.mu = math.floor((self.heater_max_temp-self.calibrate_temp-5)/(self.h*self.gamma))
                self.bands.append([self.calibrate_temp+self.mu*self.h,self.calibrate_temp+self.mu*self.gamma*self.h])
                self.bands.append([self.calibrate_temp-self.mu*self.h,self.calibrate_temp-self.mu*self.gamma*self.h])
                self.bands.append([self.calibrate_temp+self.mu*self.h*self.gamma,self.calibrate_temp-self.mu*self.h])
                self.phase = 3
                self.phase_call=0
                self.pwm_amps.append(min(self.target_PWM*(self.gamma+1),self.heater_max_power))
                self.pwm_amps.append(0.0)
                self.gcode.respond_info("max target temp: %f min target temp: %f" % (self.bands[2][0],self.bands[2][1]))
                self.gcode.respond_info("h: %f" % (self.h))
                self.gcode.respond_info("steadystate PWM: %f" % (self.target_PWM))
                self.phase_temps = []
                self.phase_pwms = [[read_time, self.pwm_amps[1]]]
                self.heater.alter_target(self.bands[1][0])
            else:
                self.phase_temps.append(temp)
            
	
            
        elif self.phase == 3:                 #in this phase, the system runs the asymetric relay and records the temps and times for each half cycle. After 2 complete cycles, the system checks if both up and down halfcycles have a percent difference of <= 0.01 between the last two cycles
            if self.phase_call == 0:
                self.gcode.respond_info("phase: %d" % (self.phase))
                self.phase_call=1
            if not self.heating and temp <= self.bands[1][0]: #if not heating and in lower band
                self.phase_temps.append([read_time, temp])
                logging.info(self.phase_temps)
                for samples in self.phase_temps:
                    if samples[0] >= self.phase_pwms[-1][0] and samples[0] <= read_time:
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[-1] > self.bands[2][0] and len(self.phase_pwms)>1:
                    self.bands[0][1]=(self.bands[2][0]/self.sampling[-1])*self.bands[0][1]
                if self.sampling[-1] < self.bands[2][0] and len(self.phase_pwms)>1:
                    self.bands[0][1]=(self.bands[2][0]/self.sampling[-1])*self.bands[0][1]
                self.sampling=[]
                self.heating= True
                self.set_pwm(read_time,self.pwm_amps[0])
                self.heater.alter_target(self.bands[0][1])
                self.phase_pwms.append((read_time,self.pwm_amps[0]))
                self.halfcycles[1].append([self.phase_pwms[-1][0]-self.phase_pwms[-2][0],self.phase_pwms[-2][1]])
                self.gcode.respond_info("up time: %f" % (self.halfcycles[-1]))
                if len(self.halfcycles[0]) >= 2 and len(self.halfcycles[1]) >= 2:
                    self.gcode.respond_info("cycle dev: %f" % (abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0])))
                    self.gcode.respond_info("up diff: %f down diff: %f" % (abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0],abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0]))
                    #if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                    if abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0] <=0.01 and abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0] <=0.01:
                        self.phase = 4
                        self.phase_call=0
                
            elif self.heating and temp >= self.bands[0][1]: #if heating and in upper band
                self.phase_temps.append([read_time, temp])
                for samples in self.phase_temps:
                    if samples[0] >= self.phase_pwms[-1][0] and samples[0] <= read_time:
                        self.sampling.append(samples[1])
                self.sampling.sort()
                if self.sampling[0] < self.bands[2][1] and len(self.phase_pwms)>1:
                    self.bands[1][0]=(self.bands[2][1]/self.sampling[0])*self.bands[1][0]
                if self.sampling[0] > self.bands[2][1] and len(self.phase_pwms)>1:
                    self.bands[1][0]=(self.bands[2][1]/self.sampling[0])*self.bands[1][0]
                self.sampling=[]
                self.heating= False
                self.set_pwm(read_time,self.pwm_amps[1])
                self.heater.alter_target(self.bands[1][0])
                self.phase_pwms.append((read_time,self.pwm_amps[1]))
                self.halfcycles[0].append([self.phase_pwms[-1][0]-self.phase_pwms[-2][0],self.phase_pwms[-2][1]])
                self.gcode.respond_info("down time: %f" % (self.halfcycles[-1]))
                if len(self.halfcycles[0]) >= 2 and len(self.halfcycles[1]) >= 2:
                    self.gcode.respond_info("cycle dev: %f" % (abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0])))
                    self.gcode.respond_info("up diff: %f down diff: %f" % (abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0],abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0]))
                    #if abs((self.halfcycles[0][-1][0]+self.halfcycles[1][-1][0]) - (self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]))/(self.halfcycles[0][-2][0]+self.halfcycles[1][-2][0]) <=0.01:
                    if abs(self.halfcycles[0][-1][0]-self.halfcycles[0][-2][0])/self.halfcycles[0][-2][0] <=0.01 and abs(self.halfcycles[1][-1][0]-self.halfcycles[1][-2][0])/self.halfcycles[1][-2][0] <=0.01:
                        self.phase = 4
                        self.phase_call=0
                        
            else: #if not at a switching point
                self.phase_temps.append([read_time, temp])
                if self.heating:
                    self.set_pwm(read_time,self.pwm_amps[0])
		else:
                    self.set_pwm(read_time,self.pwm_amps[1])

        elif self.phase == 4:
            if self.phase_call == 0:
                self.gcode.respond_info("phase: %d" % (self.phase))
                self.phase_call=1
            self.set_pwm(read_time,0.)
            self.heating= False
 
       
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        if self.heating or self.phase != 4:
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
        logging.info("halfcycles: %a", self.halfcycles)
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
        rho = max(self.halfcycles[0][-1][0],self.halfcycles[1][-1][0])/ min(self.halfcycles[0][-1][0],self.halfcycles[1][-1][0])
        tau= 1-(self.gamma - rho)/((self.gamma-1)*(0.35*rho+0.65))
        self.tau=tau
        self.rho=rho
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
        self.gcode.respond_info("rho: %f tau: %f pwmInt: %f tempInt: %f Kp: %f" %(rho,tau,pwmInt,tempInt,Kp))
        self.gcode.respond_info("cycle temps: %a" % (cycleTemps))
        T=self.halfcycles[0][-1][0]/math.log(((self.h/Kp) - self.target_PWM + math.exp(tau/(1-tau))*(self.target_PWM+self.halfcycles[0][-1][1]))/(self.halfcycles[0][-1][1]-(self.h/Kp)))
        L=T*(tau/(1-tau))
        K=(0.2*L+0.45*T)/(Kp*L)
        Ti=L*(0.4*L+0.8*T)/(L+0.1*T)
        Td=0.5*L*T/(0.3*L+T)
        self.gcode.respond_info("T: %f L: %f K: %f Ti: %f Td: %f" % (T,L,K,Ti,Td))
        Kp=K*255
        Ki=255*K/Ti
        Kd=K*Td*255
        return Kp, Ki, Kd
        
    # Offline analysis helper
    def write_file(self, filename):
        stats = ["h: %f" % (self.h),"variance: %f" % (self.vary),"threshold: %f" % (self.threshold),"mu: %f" % (self.mu),"steadystate pwm: %f" % (self.target_PWM),"max target temp: %f min target temp: %f" % (self.bands[2][0],self.bands[2][1]),"rho: %f" % (self.rho),"tau: %f" % (self.tau)]
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        f = open(filename, "wb")
        f.write('\n'.join(stats + pwm + out))
        f.close()
        
        
def load_config(config):
    return KAPAPIDCalibrate(config)

