/*
 * Power Meter for Dual Directional Coupler with LTC5507
 
 * Copyright (c) 2012 jeff millar, wa1hco
 * Distributed under the GNU GPL v2. For full terms see COPYING at the GNU website
 * used the bar graph example by Anachrocomputer that was posted at  
 *   http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1233339330
 * Functions
 *  Measure Voltage produce by LTC5507
 *  Convert Voltage to Power in Watts
 *  Display on LCD
 *    Numerical Power with short averaging and peak hold
 *    Bar graph with attack and decay dynamics
 *    Peak exceeded indication indication
 *  Bar Graphs implement industry standard hold/decay times
 *  Calibration and Configuration saved in EEPROM
 *   Directional couplers gains
 *   Bar graph limits
 *   Peak power limits, fwd/rev, for indicator
 *   Backlight Level
 *  Display
 *    Forward Power
 *    Reflected Power
 *  Inputs to Controller
 *    Forward Power DC from coupler with LTC5507
 *    Reverse Power DC from coupler with LTC5507
 *  Digital Inputs to Controller
 *    None
 *  Outputs from Controller
 *    Everything on the LCD
 *  Buttons
 *    Select, Left, Up, Down, Right
 *    Select start process, times out after 5 seconds
 *      on timeout, don't write EEPROM
 *      On select to return to power display, write EEPROM
 *    Left/Right scrolls through calibration/configuration modes
 *    Up/Down sets values of Parameters
 *  EEPROM Parameters
 *    Fwd Cal
 *    Rev Cal
 *    Display Brightness
 *    Bar graph charge/discharge time constants
 *    Numerical power peak hold time
 *    Forward/Reverse peak inidcation limit
 */

//  Power Display Layout 
//     0123456789012345 
//  0  F 2000********
//  1  R 2000*****

//  Control Display Layout 
//     0123456789012345 
//  0  Fwd Cplr -40.0dB
//  1  Pwr Fwd 200W  

// TODO:
//    change coupler values from neg gain to pos loss????
//    Check timing of 2 ms IRQ and 50 ms Display update
//    Move Control, Power Mode enum into DisplayMachine, needs to initialize enum

// include the library code:
#include           <math.h>
#include  <LiquidCrystal.h>
#include  <avr/interrupt.h>
#include       <MsTimer2.h>
#include     <avr/eeprom.h>

// Function Prototypes
// Prototpye for external Watts function written by Matlab
float Watts( float CouplerGainFwdDB, float Vol, float V );

// Function prototypes used for building function pointer table
void          FwdCalControl(int, int);  // gain of coupler (negative)
void          RevCalControl(int, int);  // gain of coupler (negative)
void       BacklightControl(int, int);  // 0 to 100%
void        FwdLimitControl(int, int);  // Watts, when to indicate
void        RevLimitControl(int, int);  // Watts, when to indicate
void     BarScaleFwdControl(int, int);  // Max scale on bar graph
void     BarScaleRevControl(int, int);  // Max scale on bar graph
void NumbersHoldTimeControl(int, int);  //

// Define SainSmart LCD 1602
const int RowPerDisplay =  2;
const int    CharPerRow = 16;
const int  PixelPerChar =  6; // including blank between characters

// LiquidCrystal(rs, enable, d4, d5, d6, d7) 
// LCD 1602 also controls backlight on Pin 10
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD1602

// Available Digital Pins Map
// Digital:    0 1 2 3 4 5 6 7 8 9 10 11 12 13  // All Digital Pins
// LCD:                4 5 6 7 8 9 10           // LCD I/F, Backlight on 10           
// PWM:              3   5 6     9 10 11        // PWM capable pins
// Avail:      0 1 2 3                11 12 13  // available

const int BacklightPin = 10;  // high for backlight on

// All the analog inputs have the same structure
// Not all analog inputs use all values
// volatile because shared between interrupt and background display context
struct analog_t 
{
           int   Pin;         // Arduino pin number
  volatile int   iAdc;        // ADC output in count from 0 to AdcMaxCount
  volatile int   iAdcAvg;     // Average value per time constant
  volatile int   iAdcPeak;    // Peak for holding numerical display
           float fVolts;      // fCal * iAdc
           float fVoltsAvg;   // IIR filtered average
           float fWattsBar;      // Power after curve fit
           float fWattsNum;
           float fCal;        // Multipy by iAdc reading to give fVolts
           float DisplayNow;  // The value on the display at the moment
           int   DisplayTime; // Time elaspsed since last display change
};

// Bar dynamics...operates on Watts
// Set UpCoef to about 5-10 msec to smooth over instantaneous peaks
// Set DecayCoef to about 250 msec to create some bar graph hang time
volatile float fAdcUpCoef;     // Time since peak updated
volatile float fAdcDecayCoef;  // Time since peak updated

// Define the analog inputs
struct analog_t PwrRev;       // Power, Reverse from sampling
struct analog_t PwrFwd;       // Power, Forward from sampling

// button press processing
static int ButtonPressTime = 0;  // msec, starts at 0, how long pressed

enum button_t
{
  NoButton,
  SelectButton,
  LeftButton, 
  RightButton, 
  UpButton, 
  DownButton,
} ButtonPressed = NoButton;      // which button pressed

// globals related to interrupt processing
const int     TimeBetweenInterrupts =  2;  // msec, time calls to ISR
const int TimeBetweenDisplayUpdates = 50;  // msec, time calls to Display and Button update

int IsrTime =  0;  // test variable, ISR execution time in msec

// Flag set at TimeBetweenDisplayUpdates by ISR, read by loop()
boolean DisplayFlag = false; 

// define the LTC5507 offsets, measured at setup, used in curve fit
float OffsetVoltFwd;
float OffsetVoltRev;

// Nonvolatile configuration settings
// read on startup
// used by EEPROM read, write routines restore, save settings
// TODO: create a dBm display and bar graph
struct settings_t
{
  float CouplerGainFwdDB;  // -10 to -60 dB
  float CouplerGainRevDB;  // -10 to -60 dB
  float BacklightLevel;    // 0 to 100%
  float LimitRev;          // 0 to 300 Watts
  float LimitFwd;          // 0 to 2000 Watts
  int   BarTypeIndex;    // {Watts, Square Law, dBm} (only Watts used, so far)
  float BarScaleFwd;     // 1 to 5000 Watts
  float BarScaleRev;     // 1 to 5000 Watts
  float BarAvgTc;      // 0 to 1000 msec
  float BarDecayTc;      // 0 to 2000 msec
  float LimitHoldTime;     // 0 to 1000 msec
} Settings;  

char LcdString[17];  // for sprintf, include null term, needed ???

//*********************************************************************************
// Entered at TimeBetweenInterrupt intervals, 
// Called from interrupt context, ADC reads occur at 2 ms, 500 Hz
// read both ADC input close together in time
// then do the peak processing
// iAdc: latest reading
// iAdcAvg: Bar graph value, smoothed with IIR filter, programmable Up/Dn TC
// iAdcPeak: numerical value, peaks held for programmable time
void UpdateAnalogInputs() 
{
  // read from the ADC twice, close together in time
  PwrFwd.iAdc = analogRead(PwrFwd.Pin); // read the raw ADC value 
  PwrRev.iAdc = analogRead(PwrRev.Pin); // read the raw ADC value 
  
  // Create a test input
  PwrFwd. iAdc = 0x3ff & (int) millis() >> 3 ;
  PwrRev. iAdc = 0x3ff & (int) millis() >> 3 ;

  // IIR filter (1-a) * history + a * new
  // a = 1/(TC / IRQ interval), calculated on Tc change, 
  // increasing time constant different from decreasing time constant
  
  // computing the IIR filters on Fwd and Rev here in interrupt context
  // able to have more time precision filtering ADC values
  if(PwrFwd.iAdc > PwrFwd.iAdcAvg)
    {
      PwrFwd.iAdcAvg = (int)(PwrFwd.iAdc* +fAdcUpCoef + PwrFwd.iAdcAvg*(1.0-fAdcUpCoef));
    }
  else if(PwrFwd.iAdc < PwrFwd.iAdcAvg)
    {
      PwrFwd.iAdcAvg = (int)(PwrFwd.iAdc* -fAdcDecayCoef + PwrFwd.iAdcAvg*(1.0-fAdcDecayCoef));
    }
  
  if(PwrRev.iAdc > PwrRev.iAdcAvg)
    {
      PwrRev.iAdcAvg = (int)(PwrRev.iAdc* +fAdcUpCoef + PwrRev.iAdcAvg*(1.0-fAdcUpCoef));
    }
  else if(PwrRev.iAdc < PwrRev.iAdcAvg)
    {
      PwrRev.iAdcAvg = (int)(PwrRev.iAdc* -fAdcDecayCoef + PwrRev.iAdcAvg*(1.0-fAdcDecayCoef));
    }
  
  static int NumberPeakTimer = 0;
  if(PwrFwd.iAdc > PwrFwd.iAdcPeak)
    {
      PwrFwd.iAdcPeak = PwrFwd.iAdc;
      NumberPeakTimer = 0;
    }
  else  // Peak Holding
    {
      NumberPeakTimer += TimeBetweenInterrupts;
      if( NumberPeakTimer > Settings.LimitHoldTime) // Peak held long enough
      {
        PwrFwd.iAdcPeak = PwrFwd.iAdc;
        NumberPeakTimer = Settings.LimitHoldTime;
      }
    }  
   if(PwrRev.iAdc > PwrRev.iAdcPeak)
    {
      PwrRev.iAdcPeak = PwrRev.iAdc;
      NumberPeakTimer = 0;
    }
  else  // Peak Holding
    {
      NumberPeakTimer += TimeBetweenInterrupts;
      if( NumberPeakTimer > Settings.LimitHoldTime) // Peak held long enough
      {
        PwrRev.iAdcPeak = PwrRev.iAdc;
        NumberPeakTimer = Settings.LimitHoldTime;
      }
    }    
  // if(iAdc > Limit) display '*' at char position 15  

} // UpdateAnalogInputs()

//*********************************************************************************
// Different averaging time constants for charging vs discharging
// if iAdc > iAdcAvg, use charging time constant
//   if charging TC set to 1, then equivalent to peak hold
// if iAdc < iAdcAvg, use discharging time constant
//   discharging set to longer time constants and can use slower calculatin interval

//********************************************************************************
// Enter at TimeInterval specified in global
// calls UpdateAnalogInputs() to read ADCs
// Sets DisplayFlag when it's time to update Display
void TimedService() 
{
  static int DisplayTimeCounter = 0;
  // debug
  IsrTime = micros(); 
  // set a flag periodically to run display update
  DisplayTimeCounter += TimeBetweenInterrupts;
  if (DisplayTimeCounter > TimeBetweenDisplayUpdates) 
  {
    DisplayTimeCounter = 0;
    DisplayFlag = true;
  }
  
  UpdateAnalogInputs();

  // debug
  IsrTime = micros() - IsrTime;
}

//*********************************************************************
// Smooth Display so low order digits don't flicker
// update display if changing or time elapsed
// if (new - current) * time since last change > delay then update
void SmoothDisplay(struct analog_t *sig, float value) 
{
  const float DisplayDelay = 300.0;  // selected by experiment
  sig->DisplayTime += TimeBetweenDisplayUpdates;
  if ( abs(value - sig->DisplayNow)*sig->DisplayTime > DisplayDelay ) {
    sig->DisplayNow = value;
    sig->DisplayTime=0;
  }
}

//**********************************************************************
// Calculate Power 
//  Fills in Fwd and Rev Power global variables
//  reads current peak power from ADC and writes Watts
void CalculatePower()
{
  volatile int iAdcLocalAvg;  // written by ISR, read in background
  
  // PwrFwd  Power forward
  noInterrupts();
  iAdcLocalAvg  = PwrFwd.iAdcAvg;
  interrupts(); 

  PwrFwd.fVoltsAvg  = PwrFwd.fCal * iAdcLocalAvg;
  PwrFwd.fWattsBar = Watts(Settings.CouplerGainFwdDB, OffsetVoltFwd, PwrFwd.fVoltsAvg - OffsetVoltFwd); 
  
  // Mask interrupts to prevent reading while changing
  noInterrupts();
  iAdcLocalAvg  = PwrRev.iAdcAvg; 
  interrupts();

  PwrRev.fVoltsAvg  = PwrRev.fCal * iAdcLocalAvg;
  PwrRev.fWattsBar = Watts(Settings.CouplerGainRevDB, OffsetVoltRev, PwrRev.fVoltsAvg - OffsetVoltRev); 
  
  // Calculate power for the Number Display, which had a peak hold function
  noInterrupts();
  PwrFwd.fVolts  = PwrFwd.fCal * PwrFwd.iAdcPeak;
  PwrRev.fVolts  = PwrRev.fCal * PwrFwd.iAdcPeak;
  interrupts(); 
  
  PwrFwd.fWattsNum = Watts(Settings.CouplerGainFwdDB, OffsetVoltFwd, PwrFwd.fVolts - OffsetVoltFwd); 
  PwrRev.fWattsNum = Watts(Settings.CouplerGainRevDB, OffsetVoltRev, PwrRev.fVolts - OffsetVoltRev); 
} // CalculatePower()

//******************************************************************************
// drawbar -- draw a bar on one row of the LCD
//   Place, starting point for bar gragh
//   row, row location for bar graph
//   ana, analog value on scale 0 to 1023
void drawbar (int StartCharLoc, int row, int ana)
{
  int bar;    // in columns, inlcude  blank column between characters
  int i;

  lcd.setCursor (StartCharLoc, row);  
  
  if(ana > 1023) ana = 1023;
  if(ana <    0) ana =    0;
  
  // Bar length set by Character count and width; analog range is 0..1023
  bar = (long)ana * (long)(CharPerRow-StartCharLoc)*PixelPerChar / 1023;
  
  // Each character cell represents six pixels
  for (i = 0; i < bar / PixelPerChar; i++)
    lcd.write(0x05);

  // Display last few pixels using user-defined characters (if not at extreme right)
  if ( i < CharPerRow) {
    lcd.write(bar % PixelPerChar);
    i++;
  }
  
  // Clear remainder of row
  for ( ; i < CharPerRow; i++)
    lcd.print((char)0x00);
}

//**********************************************************************
// Display power
// This called from background, not ISR
// Convert to floating point, calibrate, display
// Manage peak hold and smoothing update rates
// converting ADC int to calibrated float takes 60 usec
// dBm to W conversion takes 380 usec
// called from main loop
//------------------------------------------------------
void DisplayPower() 
{
  static int iWattsFwdPeak = 0;
  static int iWattsRevPeak = 0;
  static int FwdHoldTime = 0;
  static int RevHoldTime = 0;
  
  // Forward: Numbers, Bar graph, Alert processing
  CalculatePower();  // converts ADC values to Watts, Fwd and Rev, Number and Bar variants
  
  // SmoothDisplay(&PwrFwd, iWattsFwdPeak);
  lcd.setCursor(0,0);
  lcd.print("F ");
  sprintf(LcdString, "%4d", (int) PwrFwd.fWattsNum);
  lcd.print(LcdString);

  drawbar(6, 0, (int)(PwrFwd.fWattsBar/Settings.BarScaleFwd * 1023));  

 
  //SmoothDisplay(&PwrRev, iWattsRevPeak);
  
  lcd.setCursor(0,1);
  lcd.print("R ");
  sprintf(LcdString, "%4d", (int) PwrFwd.fWattsNum);
  lcd.print(LcdString);

  drawbar(6, 1, (int)(PwrRev.fWattsBar/Settings.BarScaleRev * 1023));
  
} //Display Values

//*******************************************************************
// Settings.MeterScale Fwd, Rev: 1 to 5000W
// Called at display update time

// See: http://sound.westhost.com/project55.htm 
// Peak Program Meter:  
//   A standard PPM has a 5ms integration time, so that only peaks wide 
//   enough to be audible are displayed. This translates into a response that is 
//     1dB down from a steady state reading for a 10ms tone burst, 
//     2dB down for a 5ms burst, and 
//     4dB down for a 3 ms burst. 
//   These requirements are satisfied by an attack time constant of 1.7ms. 
//   The decay rate of 1.5 seconds to a -20dB level (IEC specified) is met using a 650 ms time constant.
// VU:  
//   A VU meter is designed to have a relatively slow response. It is driven from a 
//   full-wave averaging circuit defined to reach 99% full-scale deflection in 300ms and 
//   overshoot not less than 1% and not more than 1.5%. 
//   Since a VU meter is optimised for perceived loudness it is not a good indicator of 
//   peak (transient) performance. 
//   Nominal sensitivity for 0VU is 1.228V RMS, and the impedance is 3.9k. 
// Also see http://en.wikipedia.org/wiki/Peak_programme_meter   
//   Type II PPMs fall back 24 dB in 2.8 seconds.

// from: http://www.cypress.com/?docID=33575
// Vlp/Vin = 1 / (a + z^-1*(1-a))       Voltage Gain response
// Fo = Fs/(2*pi*a), a = Fs/(2*pi*Fo)   Frequency response
// s[n] = 1-(1-(1/a))^n                 step response
// a = N/2, for a moving average of N samples, IIR of a/2 works similarly

//******************************************************************************
// monitor available, read command, output results
//-----------------------------------------
void serial()
{
  
}

//******************************************************************************
//******************************************************************************
//Group of Control function handlers
// Called with Up or Down Button Pressed
// Change between Power Display and Control Display handled externally
// Change between Control modes handles externally
// Display update handed after returning
//******************************************************************************
void FwdCalControl(int ButtonPressed, int ButtonPressTime)
{
  // update display on every pass to keep Watts Display up to date
  const float CouplerGainFwdDBMin = -60;
  const float CouplerGainFwdDBMax = -10;

  // Check range, works for up/down, no button, EEPROM initialization
  if (Settings.CouplerGainFwdDB < CouplerGainFwdDBMin )
    { Settings.CouplerGainFwdDB = CouplerGainFwdDBMin; }
  if (Settings.CouplerGainFwdDB > CouplerGainFwdDBMax )
    { Settings.CouplerGainFwdDB = CouplerGainFwdDBMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Still pressed from entering Control mode
    case LeftButton:        // Still pressed
    case RightButton:       // Still pressed
      break;
    case UpButton:
      Settings.CouplerGainFwdDB += 0.1 * .001 * ButtonPressTime;
      break;
    case DownButton:          
      Settings.CouplerGainFwdDB -= 0.1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)
  
  // first line shows directional coupler
  lcd.setCursor(0,0);
  lcd.print("Fwd Cplr ");
  lcd.print(Settings.CouplerGainFwdDB, 1);
  lcd.print("dB      ");
  // second line shows current Power level
  lcd.setCursor(0,1);
  lcd.print("Fwd Pwr ");
  CalculatePower();
  sprintf(LcdString, "%4d", (int) PwrFwd.fWattsBar);
  lcd.print(LcdString);
  lcd.print("W       ");
}

//******************************************************************************
void RevCalControl(int ButtonPressed, int ButtonPressTime)
{
  const float CouplerGainRevDBMin = -60;
  const float CouplerGainRevDBMax = -10;

  // Check range, works for up/down, no button, EEPROM initialization
  if (Settings.CouplerGainRevDB < CouplerGainRevDBMin)
    { Settings.CouplerGainRevDB = CouplerGainRevDBMin; }     
  if (Settings.CouplerGainRevDB > CouplerGainRevDBMax)
    { Settings.CouplerGainRevDB = CouplerGainRevDBMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:            
      Settings.CouplerGainRevDB += 0.1 * .001 * ButtonPressTime;
      break;
    case DownButton:
      Settings.CouplerGainRevDB -= 0.1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  //  First line, coupler gains
  lcd.setCursor(0,0);
  lcd.print("Rev Cplr ");
  lcd.print(Settings.CouplerGainRevDB, 1);
  lcd.print("dB      ");
  // Second line, current reflected power
  lcd.setCursor(0,1);
  lcd.print("Rev Pwr ");
  CalculatePower();
  sprintf(LcdString, "%4d", (int) PwrRev.fWattsBar);
  lcd.print(LcdString);
  lcd.print("W       ");
}

//******************************************************************************
// called in Control Mode for any button press
void BacklightControl(int ButtonPressed, int ButtonPressTime)
{
  // handle corrupt or unitialized EERPOM
  const int   BacklightLevelMin =   0;
  const int   BacklightLevelMax = 100;

  // Check range, works for up/down, no button, and EEPROM initialization
  if (Settings.BacklightLevel < BacklightLevelMin) 
    { Settings.BacklightLevel = BacklightLevelMin; }
  if (Settings.BacklightLevel > BacklightLevelMax) 
    { Settings.BacklightLevel = BacklightLevelMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:
      Settings.BacklightLevel += 1 * .001 * ButtonPressTime;
      break;
    case DownButton:
      Settings.BacklightLevel -= 1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  analogWrite(BacklightPin, (int)(Settings.BacklightLevel*255)/100);  // Set backlight brightness              

  lcd.setCursor(0,0);
  lcd.print("Backlight ");
  lcd.print(((int) Settings.BacklightLevel));
  lcd.print("%          ");
  lcd.setCursor(0,1);
  lcd.print("                ");
} // BacklightControl()

//******************************************************************************
// Control for the Forward Power limit
//  Check on range
//  Up, Down button processing with hold time acceleration
//  Display
void FwdLimitControl(int ButtonPressed, int ButtonPressTime)
{
  // handle corrupt or unitialized EERPOM
  const int   LimitFwdMin =   10;
  const int   LimitFwdMax = 1500;
  
  // Check range, works for up/down, no button, EEPROM initialization
  if (Settings.LimitFwd < LimitFwdMin)
    { Settings.LimitFwd = LimitFwdMin; } 
  if (Settings.LimitFwd > LimitFwdMax)
    { Settings.LimitFwd = LimitFwdMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Still pressed from entering Control mode
    case LeftButton:        // Still pressed
    case RightButton:       // Still pressed
      break;
    case UpButton:
      Settings.LimitFwd += 1 * .001 * ButtonPressTime;
     break;
    case DownButton:          
      Settings.LimitFwd -= 1 * .001 * ButtonPressTime;
     break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Fwd Limit ");
  lcd.print( (int) Settings.LimitFwd );
  lcd.print("W         ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

//******************************************************************************
// Control for the Reflected Power limit
//  Check on range
//  Up, Down button processing with hold time acceleration
//  Display
void RevLimitControl(int ButtonPressed, int ButtonPressTime)
{
  // handle corrupt or unitialized EERPOM 
  const int   LimitRevMin =  10;
  const int   LimitRevMax = 300;

  // Check range, works for up/down, no button, EEPROM initialization
  if (Settings.LimitRev < LimitRevMin)
    { Settings.LimitRev = LimitRevMin; }     
  if (Settings.LimitRev > LimitRevMax)
    { Settings.LimitRev = LimitRevMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:            
      Settings.LimitRev += 1 * .001 * ButtonPressTime;
      break;
    case DownButton:
      Settings.LimitRev -= 1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Rev Limit ");
  lcd.print( (int) Settings.LimitRev );
  lcd.print("W        ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

//******************************************************************************
// Define the Bar Scale
void BarScaleFwdControl(int ButtonPressed, int ButtonPressTime)
{
  const int BarScaleMin = 1;
  const int BarScaleMax = 5000;  

  // Limit range to between Min and Max
  if( Settings.BarScaleFwd < BarScaleMin) 
    { Settings.BarScaleFwd = BarScaleMin; }   
  if( Settings.BarScaleFwd > BarScaleMax)
    { Settings.BarScaleFwd = BarScaleMax; }   
  
  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      Settings.BarScaleFwd += 1 * .001 * ButtonPressTime;
      break;
    case DownButton:            
      Settings.BarScaleFwd -= 1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Bar Fwd Max ");
  lcd.setCursor(0,1);
  lcd.print(Settings.BarScaleFwd, 0 );
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Bar Scale
void BarScaleRevControl(int ButtonPressed, int ButtonPressTime)
{
  const int BarScaleMin = 1;
  const int BarScaleMax = 5000;  
  
  // Limit range to between Min and Max
  if( Settings.BarScaleRev < BarScaleMin ) 
    { Settings.BarScaleRev = BarScaleMin; }   
  if( Settings.BarScaleRev > BarScaleMax )
    { Settings.BarScaleRev = BarScaleMax; }   

  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:
        Settings.BarScaleRev += 1 * .002 * ButtonPressTime;
      break;
    case DownButton:            
        Settings.BarScaleRev -= 1 * .002 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Bar Rev Max ");
  lcd.setCursor(0,1);
  lcd.print(Settings.BarScaleRev, 0);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Bar Averaging Time Constant
void BarAvgTcControl(int ButtonPressed, int ButtonPressTime)
{
  const float BarAvgTcMin =    1.0;
  const float BarAvgTcMax = 2000.0;  
  
  // Limit range to between Min and Max
  if( Settings.BarAvgTc < BarAvgTcMin ) 
    { Settings.BarAvgTc = BarAvgTcMin; }   
  if( Settings.BarAvgTc > BarAvgTcMax )
    { Settings.BarAvgTc = BarAvgTcMax; }   
  
  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      if(ButtonPressTime == 0)
        { Settings.BarAvgTc += 1.0; }
      else
        { Settings.BarAvgTc += 1.0 * .001 * ButtonPressTime; }
      break;
    case DownButton:            
      if(ButtonPressTime == 0)
        { Settings.BarAvgTc -= 1.0; }
      else
        { Settings.BarAvgTc -= 1.0 * .001 * ButtonPressTime; }
      break;
  } // switch(ButtonPressed)

  // compute iAdcIIR coefficient, 
  // iAdcAvg = (1-iAdcIIR) * iAdcAvg + iAdcIIR * iAdc
  // BarAvgTc and TimerBetweenInterrupts in msec
  fAdcUpCoef = 1.0/( Settings.BarAvgTc / (float) TimeBetweenInterrupts );

  lcd.setCursor(0,0);
  lcd.print("Bar Avg ms   ");
  lcd.setCursor(0,1);
  lcd.print(Settings.BarAvgTc, 0);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Bar Decay Time Constand
//
void BarDecayTcControl(int ButtonPressed, int ButtonPressTime)
{
  const float BarDecayTcMin = 1.0;
  const float BarDecayTcMax = 2000.0;  
  
  // Limit range to between Min and Max
  if( Settings.BarDecayTc < BarDecayTcMin ) 
    { Settings.BarDecayTc = BarDecayTcMin; }   
  if( Settings.BarDecayTc > BarDecayTcMax )
    { Settings.BarDecayTc = BarDecayTcMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      if(ButtonPressTime == 0)
        { Settings.BarDecayTc += 1; }
      else
        { Settings.BarDecayTc += 1 * .001 * ButtonPressTime; }
      break;
    case DownButton:            
      if(ButtonPressTime == 0)
        { Settings.BarDecayTc -= 1; }
      else
        { Settings.BarDecayTc -= 1 * .001 * ButtonPressTime; }
      break;
  } // switch(ButtonPressed)

  // compute iAdcIIR coefficient, 
  // iAdcAvg = (1-iAdcIIR) * iAdcAvg + iAdcIIR * iAdc
  // BarAvgTc and TimerBetweenInterrupts in msec
  fAdcDecayCoef = 1.0/( Settings.BarDecayTc / (float) TimeBetweenInterrupts );

  lcd.setCursor(0,0);
  lcd.print("Bar Decay ms   ");
  lcd.setCursor(0,1);
  lcd.print( Settings.BarDecayTc, 0 );
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Bar Decay Time Constand
//
void NumbersHoldTimeControl(int ButtonPressed, int ButtonPressTime)
{
  const float LimitHoldTimeMin = 0.0;
  const float LimitHoldTimeMax = 1000.0;  
  
  // Limit range to between Min and Max
  if( Settings.LimitHoldTime < LimitHoldTimeMin ) 
    { Settings.LimitHoldTime = LimitHoldTimeMin; }   
  if( Settings.LimitHoldTime > LimitHoldTimeMax )
    { Settings.LimitHoldTime = LimitHoldTimeMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      if(ButtonPressTime == 0)
        { Settings.LimitHoldTime += 1.0; }
      else
        { Settings.LimitHoldTime += 1.0 * .001 * ButtonPressTime; }
      break;
    case DownButton:            
      if(ButtonPressTime == 0)
        { Settings.LimitHoldTime -= 1.0; }
      else
        { Settings.LimitHoldTime -= 1.0 * .001 * ButtonPressTime; }
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Number Peak Hold");
  lcd.setCursor(0,1);
  lcd.print( Settings.LimitHoldTime, 0 );
  lcd.print(" msec           ");  // finish out the line 
}

//***********************************************************************
void ProcessPowerDisplay()
{
  DisplayPower();      // LCD numbers, P.fwd and P.rev   
}

//******************************************************************************
// state machine, Entered at timed intervals
// switch on Control vs Power Display mode
// Select button change between Control and Power mode
//   Power Mode: Select Pressed
//     ButtonPressTime == 0: Switch to Control on press, Display Settings
//     ButtonPressTime > 0; Continue to display "Settings"
//   Control Mode: Select Pressed
//     ButtonPressTime == 0: Switch to Power mode
//   Control Mode: Select switches to Power mode on press, holding press causes write to EEPROM
// Left and Right buttons change the control mode
// Up and Down handled in the Control routines
// then call ProcessControlMode(ModeIndex) which branches to control routines
void DisplayMachine()
{ 
  // List of all states
  enum state_t 
  { PowerMode, ControlMode };
  static state_t DisplayState = PowerMode;
  
  static int EepromHoldTimer = 0;
  
  // define type for a pointer to a control function
  typedef void (*fControlPtr)(int, int);
  // build an array of control routine pointers.
  fControlPtr fControl[11] = 
  { 
             FwdCalControl,
             RevCalControl,
          BacklightControl,
           FwdLimitControl,
           RevLimitControl,
        BarScaleFwdControl,
        BarScaleRevControl,
           BarAvgTcControl,
         BarDecayTcControl,
     NumbersHoldTimeControl
  };
  
  const char            FwdMode =  0;
  const char            RevMode =  1;
  const char      BacklightMode =  2;
  const char       FwdLimitMode =  3;
  const char       RevLimitMode =  4;
  const char    BarScaleFwdMode =  5;
  const char    BarScaleRevMode =  6;
  const char       BarAvgTcMode =  7;
  const char     BarDecayTcMode =  8;
  const char NumbersHoldTimeMode = 9;
  
  static int ModeIndex    = 0;
   const int ModeIndexMin = 0;
   const int ModeIndexMax = 9;

  ReadButton();  // sets value of ButtonPressed and ButtonPressTime

  // Entered at TimeBetweenDisplayUpdates, while Select button pressed
  // Switch modes between Power and Control when first pressed
  // While button held, display what mode switching to.
  // Press and hold going to power shows countdown to programming EERPOM
  if(ButtonPressed == SelectButton)
  {
    if(DisplayState == ControlMode)
    {
      if(ButtonPressTime == 0)  // initial press
      {
        DisplayState = PowerMode;
        EepromHoldTimer = 3000; // start transition to Power mode
        lcd.setCursor(0,0);
        lcd.print("To Power Mode   ");
      }
      // else{ } not needed
    } // DisplayState==ControlMode

    else // Power mode
    {
      if(ButtonPressTime == 0) // initial press
      {
        // switch to control mode, display 
        DisplayState = ControlMode;
        lcd.setCursor(0,0);
        lcd.print("To Control Mode ");
        lcd.setCursor(0,1);
        lcd.print("                ");
      }
      else  // Button held, transitioning to Power Mode
      {
        //  Button still held from Control Mode
        //  EepromHoldTimer initialized to 3000 msec on start of transition to PowerMode
        //    then counts down to 0
        // three states >0, near 0, << 0 
        if(EepromHoldTimer >= TimeBetweenDisplayUpdates) // EepromHoldTimer counting down
        {
          EepromHoldTimer -= TimeBetweenDisplayUpdates;
          lcd.setCursor(0,1);
          lcd.print("Set in ");
          lcd.print(EepromHoldTimer);
          lcd.print(" msec        ");
        } // eeprom hold counting down
        else if(EepromHoldTimer < TimeBetweenDisplayUpdates && EepromHoldTimer > -TimeBetweenDisplayUpdates) // near zero
        {
          EepromHoldTimer = -2 * TimeBetweenDisplayUpdates;  // 
          eeprom_write_block((const void*)&Settings, (void*)0, sizeof(Settings));
          lcd.setCursor(0,0);
          lcd.print("EEPROM Writen   ");
          lcd.setCursor(0,1);
          lcd.print("                ");
        } // near zero
        else if(EepromHoldTimer <= -TimeBetweenDisplayUpdates)
        {
          // wait in this path until button released
        }
      } // Select held from Control mode
    } // DisplayState==PowerMode
  } // ButtonPressed==Select

  else if(DisplayState == PowerMode)
  {
    // Buttons don't matter, Select button alread processed
    ProcessPowerDisplay();
  }

  else if(DisplayState == ControlMode)  // Control mode or Power Mode
  {
    switch (ButtonPressed)
    {   
      case LeftButton:
        if (ButtonPressTime == 0)    // On leading edge of button press
        {
          // previous control mode, wrap around
          if (ModeIndex > ModeIndexMin) { ModeIndex -= 1; }  
          else if (ModeIndex <= ModeIndexMin) { ModeIndex = ModeIndexMax; }       
        }
        break;  // LeftButton

      case RightButton:
        if (ButtonPressTime == 0)    // On leading edge of button press
        {
          // next control mode, wrap around
          if (ModeIndex < ModeIndexMax) { ModeIndex += 1; }  // next control mode
          else if (ModeIndex >= ModeIndexMax) { ModeIndex = ModeIndexMin; }          
        }
        break;  // RightButton
    }  // switch(ButtonPressed)
    // Call Control routine using the function point array      
    (*fControl[ModeIndex])(ButtonPressed, ButtonPressTime);  
  } // if(ControlMode)

} // DisplayMachine()

//*************************************************************************
// Called at TimeBetweenDisplayUpdates intervals (50 ms)
// Sets two global variables
//  ButtonPressed, enum of select, left, right, up, down
//  ButtonPressTime, how long held to enable timed press and hold processing
// Derived from a demo program that appears on some sites that sell the LCD1602
//**************************************************************************
void ReadButton()
{
  const int NUM_KEYS = 5;
  // buttons ground various resistor, producing different ADC values
  const int adc_key_val[5] ={50, 200, 400, 600, 800 };
  static int key=-1;    // initialize to not pressed
  static int oldkey=-1; // initialize to not pressed
  int adc_key_in;
  
  adc_key_in = analogRead(0); // read the value from the sensor 
 
  // convert into key press
  for (key = 0; key < NUM_KEYS; key++)
    {
      if ( adc_key_in < adc_key_val[key] ) { break; }
      if (key >= NUM_KEYS) { key = -1; } // No valid key pressed
    }
 
  if (key != oldkey)          // if keypress transition is detected
  {
    oldkey = key;
    ButtonPressTime = 0;
    ButtonPressed = NoButton;    // default to nobutton, override if key pressed
    if (key >= 0)        // key pressed
    {
      switch (key)
      {
        case 0:
          ButtonPressed = RightButton;
          break;
        case 1:
          ButtonPressed = UpButton;
          break;
        case 2:
          ButtonPressed = DownButton;
          break;
        case 3:
          ButtonPressed = LeftButton;
          break;
        case 4:
          ButtonPressed = SelectButton;
          break;
      } // switch(key)
    } // if(key>0)
  }
  else // key == oldkey
  {
    const int ButtonPressTimeMax = 5000;  // msec, Max how long button pressed (needed?)
    // increment button press time by time between display updates
    ButtonPressTime += TimeBetweenDisplayUpdates;
    if (ButtonPressTime > ButtonPressTimeMax)
    {
      ButtonPressTime = ButtonPressTimeMax;
    } 
  } // if(key!=oldkey)
} // ReadButton()

//*****************************************************************************
// Define custom character to support bar graph
// called only during setup, writes to the LCD controller
void defChar (LiquidCrystal &thelcd, int asc, const unsigned char row[8])
{
  int i;
  
  if ((asc < 0) || (asc > 7))
    return;
    
  thelcd.command (0x40 | (asc << 3));
  
  for (i = 0; i < 8; i++)
    thelcd.write (row[i]);
    
  thelcd.home ();
}

//*******************************************************************************
// Configure the pins, set outputs to standby
// Enable interrupts at the end
void setup() 
{
  // used to set floating point ADC calibration value
  const int   AdcMaxCount = 1023;
  const float AdcMaxVolts = 5.0;
  
  // Setup pin modes
  pinMode(BacklightPin, OUTPUT);

  // read and fill in the calibration Settings values
  eeprom_read_block( (void*)&Settings, (void*)0, sizeof(Settings) );

  analogWrite(BacklightPin, (int)(Settings.BacklightLevel*255)/100);  // Set backlight brightness              

  //  Define custom characters to support bargraph
  static unsigned char cheq[8] = {0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA};
  static unsigned char bar0[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  static unsigned char bar1[8] = {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10};  
  static unsigned char bar2[8] = {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18};
  static unsigned char bar3[8] = {0x1c,0x1c,0x1c,0x1c,0x1c,0x1c,0x1c,0x1c};
  static unsigned char bar4[8] = {0x1e,0x1e,0x1e,0x1e,0x1e,0x1e,0x1e,0x1e};
  static unsigned char bar5[8] = {0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f,0x1f};
  
  // Call the function to write the custom char to the LCD display
  lcd.clear ();
  defChar (lcd, 0x00, bar0);
  defChar (lcd, 0x01, bar1);
  defChar (lcd, 0x02, bar2);
  defChar (lcd, 0x03, bar3);
  defChar (lcd, 0x04, bar4);
  defChar (lcd, 0x05, bar5);
  defChar (lcd, 0x06, cheq);

  // set up the LCD's number of columns and rows: 
  lcd.clear();
  lcd.begin(CharPerRow, RowPerDisplay);

  lcd.setCursor(0,0);
  lcd.print("Power Meter v1.0");
  lcd.setCursor(0,1);
  lcd.print("   by WA1HCO    ");
  delay(3000);  // 3000 ms, 3 seconds
  
  // The delay above give the ADC and LTC5507 time settle down (if necessary)

  // Button pin = A0 
  PwrFwd.Pin = A1;  // input pin for forward Power
  PwrRev.Pin = A2;  // input pin for reverse Power
   
  // Power Cal start with converting to volts
  PwrFwd.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iAdc to float Volts
  PwrRev.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iAdc to float Volts

  // Read Forward and Reverse ADC values, assuming no RF to set Vol
  PwrFwd.fVolts = PwrFwd.fCal * analogRead(PwrFwd.iAdc);
  PwrRev.fVolts = PwrRev.fCal * analogRead(PwrRev.iAdc);
  
  // compute iAdcIIR coefficient, 
  // iAdcAvg = (1-iAdcIIR) * iAdcAvg + iAdcIIR * iAdc
  // BarAvgTc and TimerBetweenInterrupts in msec
  fAdcUpCoef    = 1.0/( Settings.BarAvgTc   / (float) TimeBetweenInterrupts );
  fAdcDecayCoef = 1.0/( Settings.BarDecayTc / (float) TimeBetweenInterrupts );  

  OffsetVoltFwd = PwrFwd.fVolts;
  OffsetVoltRev = PwrRev.fVolts;

  // for debug purposes
  OffsetVoltFwd = 0.6;
  OffsetVoltRev = 0.6;
 
  // The serial port 
  Serial.begin(57600);
  
  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TimeBetweenInterrupts, TimedService); // 2ms period
  // interrupts enabled after this point
  MsTimer2::start(); 
} // setup


//*****************************************************************
// Main Loop, just for display stuff
// Arduino falls into this after setup()
// Loop executes in about 14 to 19 msec
// This loop gets interrupted periodically
// All timed stuff occurs in ISR
//--------------------------------------------------------------
void loop() 
{
  // watch for signal from interrupt handler
  // DisplayFlag set in interrupt handler Timebetween 
  if (DisplayFlag == true) 
  {
    int DisplayTime;
    DisplayFlag = false;  // clear the flag
    DisplayTime = micros();
    DisplayMachine();     // state machine based on ButtonPressed
  }
}
