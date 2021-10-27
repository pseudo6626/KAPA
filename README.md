# project-KAPA

KAPA - Klipper Asymmetric PID Autotuning

KAPA is a modification of the Klipper PID autotune python file for improved performance and stability of resulting PID coefficients by utilizing asymmetric relay testing calibration methods. The new routine is based off of this work: https://portal.research.lu.se/portal/files/3633151/5463952.pdf and feautures asymetric cyclical tuning. This allows the PID to compensate for systems where the rate of increase for the desired variable of change is significantly differet from the rate of decrease. (for example, large glass 3d printing beds)

This is a WIP and as of now has barely even been started. Current roadmap includes:

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
