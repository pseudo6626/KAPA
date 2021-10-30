# project-KAPA

KAPA - Klipper Asymmetric PID Autotuning

KAPA is a modification of the Klipper PID autotune python file for improved performance and stability of resulting PID coefficients by utilizing asymmetric relay testing calibration methods. The new routine is based off of this work: https://portal.research.lu.se/portal/files/3633151/5463952.pdf and feautures asymetric cyclical tuning. This allows the PID to compensate for systems where the rate of increase for the desired variable of change is significantly differet from the rate of decrease. (for example, large glass 3d printing beds)

Update: KAPA is now in a working beta development. I will attempt here to give an ELI5ish description of how it works and why it is advantagious: 

First, calling the command is very simmilar to the stock command, but with one added input:

PID_CALIBRATE HEATER=[desired heater]  TARGET=[target temp]  GAMMA= [desired asymmetry value]  WRITE_FILE=[1 for a txt of temp values]


  The new gamma input should be an integer greater than 1 that defines the asymmetry of the experiment. Default values of 6 to 10 are recommended. NOTE: the larger your gamma, the greater the target temperatures will be during testing.  A gamma value MUST be provided.
  
  
  
  
  
KAPA opperates in 5 distinct phases from 0 to 5. Below are descriptions of each phase:

Phase 0: In this phase, the system sets the heaters to 100% until the target temperature is reached. 

Phase 1: In this phase, the system sets the heaters to 0% for 6 seconds, and then begins to raise/lower the heater's power until it finds a % that is just enough to keep the temperature at a steady state. In this case, steady state is defined as a variance of less than or equal to 0.00025c^2 over 15 sampled temps across 5 seconds. (a variance of 0.00025 correlates to a stdev of 0.0158c, which means that for 15 samples, there is a 6sigma certainty (99.999999% chance) that the temperature will no have varied by 0.1c)

Phase 2: In this phase, the system sets the heaters to the % found in phase 1 and monitors the temperature for 5 seconds. The system then uses the highest deviation measured in the temp for this time to find the hysterics (noise) level of the thermistor. From this, the system then identifies Upper and Lower bands of temperature that it will use as targets for the relay. It also identifies the ideal max and min temps for the relay cycle. Finally, the system calculates the heater % it will use to drive the "up" times in the relay cycle. In all these calculations, the upper values will be gamma times greater than the lower values. 

Phase 3: In this phase, the system will begin runing the relay program. It monitors the temperature, and turns the heater on and off whenever it enters the Upper or Lower bands. It also monitors the max and min temps, and adjusts the bands as necessary to try and get the max and min temperatures as close to the ideal values as possible.  After 2 full cycles, at each switching point, the system will check if the "up" times and "down" times for the latest cycle are within 1.0% error of the values for the last cycle. If so, the relay ends. If not, it will contune until this condition is met.

Phase 4: The system shuts heating off and notifies klipper that it should end the autotune and proceed to analysis






Current roadmap includes:

~~1) Learn how the Klipper PID python script currently functions~~

~~2) Understand the cited PID work well enought to translate it into a pseudo code~~

~~3) Generate a rough pseudo code~~

~~4) Learn enough python to inject new routine into the existing framework of the Klipper PID python script~~

~~5) trick my existing Klipper deployment on my Sidewinder X1 to run the modified PID script~~

~~6) Trial the new design and record changes in response time, stability, and performance~~

~~7) Put out any fires that arise from step 6~~

8) Publish working script to volunteers for beta testing

9) Teleconference with volunteers on best methods for extinguishing electrical fires

10) PROFIT! (but not because this isn't liscenced for that)

11) Try not to anger the Kevin.
