Power Meter for Dual Directional Coupler with LTC5507
 
Copyright (c) 2012 jeff millar, wa1hco. Distributed under the GNU GPL v2. For full terms see COPYING at the GNU website  
 
Uses the bar graph example by Anachrocomputer that was posted at  
  http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1233339330  

Processing...
- Measure Voltage produce by LTC5507 power measurement chip
  - LTC5507 is internal schottky diode RF Detector  
  - Wide Input Power Range: -34dBm to 14dBm  
  - Ultra Wide Input Frequency Range: 100kHz to 1000MHz  
- Convert Voltage to power in Watts  
  - Useing the data sheet calibration tables
- Display on 2 rows by 16 column LCD
  - Forward Power  
  - Reflected Power  
  - Numerical power uses 4 digits  
  - Bar graph with attack and decay dynamics   
- Bar Graphs implement industry standard hold/decay times  
- Calibration and Configuration saved in EEPROM  
  - Directional coupler gains  
  - Bar graph limits  
  - Peak power limits, fwd/rev, for indicator  
  - Backlight Level  
- Inputs to Controller  
  - Forward Power DC from coupler with LTC5507  
  - Reverse Power DC from coupler with LTC5507  
- Digital Inputs to Controller  
  - None  
- Outputs from Controller  
  - Everything on the LCD  
- Buttons  
  - Select, off to itself
  - Left, Up, Down, Right, arranged in diamond pattern  
  - Select starts user input process, times out after 5 seconds  
    - On timeout, don't write EEPROM  
    - On Select to return to power display, write EEPROM  
  - Left/Right scrolls through calibration/configuration modes  
  - Up/Down sets values of Parameters
- EEPROM Parameters, saved calibrations and settings  
  - Fwd Cal  
  - Rev Cal  
  - Display Brightness  
  - Bar graph charge/discharge time constants  
  - Numerical power peak hold time  
  - Forward/Reverse peak inidcation limit  
- Power Display Layout  
  - Two rows, forward and reverse
  - 4 digit numerical power
  - 10 digit bar graph using special characters
  -   F 2000********  
  -   R 2000*****  
 - Control Display Layout  
   - EEPROM parameter name, value, units
   - Fwd Cplr -40.0 dB  
   - Pwr Fwd 200 W  
- TODO:  
  - change coupler values from neg gain to pos loss????  
  - Check timing of 2 ms IRQ and 50 ms Display update  
  - Move Control, Power Mode enum into DisplayMachine, needs to initialize enum
