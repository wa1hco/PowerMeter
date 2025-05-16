 Power Meter for Dual Directional Coupler with LTC5507
 
 Copyright (c) 2012 jeff millar, wa1hco
 Distributed under the GNU GPL v2. For full terms see COPYING at the GNU website
 
 Uses the bar graph example by Anachrocomputer that was posted at  
   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1233339330
   
 Functions
  Measure Voltage produce by LTC5507
  Convert Voltage to Power in Watts
  Display on LCD
    Numerical Power with short averaging and peak hold
    Bar graph with attack and decay dynamics
    Peak exceeded indication indication
  Bar Graphs implement industry standard hold/decay times
  Calibration and Configuration saved in EEPROM
   Directional couplers gains
   Bar graph limits
   Peak power limits, fwd/rev, for indicator
   Backlight Level
  Display
    Forward Power
    Reflected Power
  Inputs to Controller
    Forward Power DC from coupler with LTC5507
    Reverse Power DC from coupler with LTC5507
  Digital Inputs to Controller
    None
  Outputs from Controller
    Everything on the LCD
  Buttons
    Select, Left, Up, Down, Right
    Select start process, times out after 5 seconds
      on timeout, don't write EEPROM
      On select to return to power display, write EEPROM
    Left/Right scrolls through calibration/configuration modes
    Up/Down sets values of Parameters
  EEPROM Parameters
    Fwd Cal
    Rev Cal
    Display Brightness
    Bar graph charge/discharge time constants
    Numerical power peak hold time
    Forward/Reverse peak inidcation limit
 
 Power Display Layout 
    0123456789012345 
    
 0  F 2000********
 
 1  R 2000*****
 

 Control Display Layout 
    0123456789012345 
 0  Fwd Cplr -40.0dB
 1  Pwr Fwd 200W  

TODO:
   change coupler values from neg gain to pos loss????
   Check timing of 2 ms IRQ and 50 ms Display update
   Move Control, Power Mode enum into DisplayMachine, needs to initialize enum
