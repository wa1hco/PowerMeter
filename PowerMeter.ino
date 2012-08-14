/*
 * Power Meter for Dual Directional Coupler with LTC5507
 
 * Copyright (c) 2012 jeff millar, wa1hco
 * Distributed under the GNU GPL v2. For full terms see COPYING at the GNU website
 
 * Functions
 *  Measure Voltage produce by LTC5507
 *  Convert Voltage to Power in Watts
 *  Display Power on LCD
 *  Drive Forward and Reflected Meters from PWM
 *  Drive Forward and Reflected Limit LEDs
 *  Meters work in a variety of modes
 *    Meters read in dBm
 *    Averaging
 *    Peak Watts
 *  Meters implement peak hold algorithm
 *  Meters implement industry standard meter hold/decay times
 *  Calibration and Configuration saved in EEPROM
 *   Directional couplers gains
 *   Meter Modes
 *   Backlight Level
 *   Forward and Reflected Limit Levels
 *  Display
 *    Forward Power
 *    Reflected Power
 *  Inputs to Controller
 *    Forward Power DC from coupler with LTC5507
 *    Reverse Power DC from coupler with LTC5507
 *  Digital Inputs to Controller
 *    None
 *  Outputs from Controller
 *    Forward Meter PWM
 *    Reverse Meter PWM
 *    LED drive on Excessive forward Power
 *    LED drive on Excessive reflected Power
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
 *    Meter Dynamics: Raw, Peak Hang, Average
 *    Forward limit
 *    Reverse Limit
 */

//  Power Display Layout 
//     0123456789012345 
//  0  PwrFwd 2000W
//  1  PwrRev 2000W  

//  Control Display Layout 
//     0123456789012345 
//  0  PwrFwd 2000W
//  1  PwrRev 2000W  

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
#include        <Firmata.h>

// Function Prototypes
// Prototpye for external Watts function written by Matlab
float Watts( float CouplerGainFwdDB, float Vol, float V );

// Function prototypes used for building function pointer table
void        FwdCalControl(int, int);
void        RevCalControl(int, int);
void     MeterModeControl(int, int);
void     BacklightControl(int, int);
void      FwdLimitControl(int, int);
void      RevLimitControl(int, int);
void     MeterTypeControl(int, int);
void MeterScaleFwdControl(int, int);
void MeterScaleRevControl(int, int);

// LiquidCrystal(rs, enable, d4, d5, d6, d7) 
// LCD 1602 also controls backlight on Pin 10
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD1602

// Available Digital Pins Map
// Digital:    0 1 2 3 4 5 6 7 8 9 10 11 12 13  // All Digital Pins
// LCD:                4 5 6 7 8 9 10           // LCD I/F, Backlight on 10           
// PWM:              3   5 6     9 10 11        // PWM capable pins
// Meter:            3                11        // Meter drive
// LED:        0 1                              // Fwd, Ref warning LED
// Avail:      0   2                     12     // available

// Digital Pins
const int     MeterPinFwd =  3;  // PWM output to Forward meter
const int     MeterPinRev = 11;  // PWM output to Reverse meter
const int     LimitPinRev =  1;  // LED Output on high reflected power
const int     LimitPinFwd = 13;  // LED Output on high peak power
const int    BacklightPin = 10;  // high for backlight on

// All the analog inputs have the same structure
// Not all analog inputs use all values
// volatile because shared between interrupt and background display context
struct analog_t 
{
           int   Pin;         // Arduino pin number
  volatile int   iAdc;        // ADC output in count from 0 to AdcMaxCount
  volatile int   iAdcPeak;    // Peak value, holds and decays
  volatile int   iAdcAvg;     // Average value per time constant
  volatile float fAdcIIR;     // Time since peak updated
           float fVolts;      // fCal * iAdc
           float fVoltsPeak;  // fCal * iAdcPeak
           float fVoltsAvg;   // IIR filtered average
           float fWatts;      // Power after curve fit
           float fCal;        // Multipy by iAdc reading to give fVolts
           float DisplayNow;  // The value on the display at the moment
           int   DisplayTime; // Time elaspsed since last display change
};

// Define the analog inputs
struct analog_t PwrRev;    // Power, Reverse from sampling
struct analog_t PwrFwd;    // Power, Forward from sampling

// button press processing
static int ButtonPressTime = 0;       // 

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
      int                   IsrTime =  0;  // test variable, ISR execution time in msec
  boolean DisplayFlag = false; 

// define the LTC5507 offsets, measured at setup, used in curve fit
float OffsetVoltFwd;
float OffsetVoltRev;

// Nonvolatile configuration settings
// read on startup
// used by EEPROM read, write routines restore, save settings
struct settings_t
{
  float CouplerGainFwdDB;  // -10 to -60 dB
  float CouplerGainRevDB;  // -10 to -60 dB
  float BacklightLevel;    // 0 to 100%
  float LimitRev;          // 0 to 300 Watts
  float LimitFwd;          // 0 to 2000 Watts
  char  MeterModeIndex;    //  
  int   MeterTypeIndex;    // {Linear, Square Law}
  float MeterScaleFwd;     // 1 to 5000 Watts
  float MeterScaleRev;     // 1 to 5000 Watts
  float MeterAvgTc;        // 0 to 1000 msec
  float MeterDecayTc;      // 0 to 2000 msec
} Settings;  

char LcdString[17];  // for sprintf, include null term, needed ???

//*********************************************************************************
// Entered at TimeIncement intervals, 
// called from interrupt context
// read both ADC input close together in time
// then do the peak processing
// ADC reads occur at 2 ms, 500 Hz
// Display processing reads values in iAdc and iAdcPeak at 50 ms, 20 Hz
//-----------------------------------------
void UpdateAnalogInputs(int DisplayTimeCounter) 
{
  // read from the ADC twice close together
  PwrFwd.iAdc = analogRead(PwrFwd.Pin); // read the raw ADC value 
  PwrRev.iAdc = analogRead(PwrRev.Pin); // read the raw ADC value 
  
  // test input
  PwrFwd. iAdc = 0x3ff & (int) millis() >> 3 ;
  PwrRev. iAdc = 0x3ff & (int) millis() >> 3 ;

  if(PwrFwd.iAdc > PwrFwd.iAdcPeak || DisplayTimeCounter == 0) // if new peak, or end of interval
  { PwrFwd.iAdcPeak = PwrFwd.iAdc; }   // save new peak

  // IIR filter (1-a) * history + a * new
  // a = 1/(TC / IRQ interval)
  PwrFwd.iAdcAvg = (1.0-PwrFwd.fAdcIIR) * PwrFwd.iAdcAvg + PwrFwd.fAdcIIR * PwrFwd.iAdc;

  if(PwrRev.iAdc > PwrRev.iAdcPeak || DisplayTimeCounter == 0) // if new peak, or end of interval
  { PwrRev.iAdcPeak = PwrRev.iAdc; }   // save new peak

  PwrRev.iAdcAvg = (1.0-PwrRev.fAdcIIR) * PwrRev.iAdcAvg + PwrRev.fAdcIIR * PwrRev.iAdc;


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
  
  UpdateAnalogInputs(DisplayTimeCounter);

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
//  reads current peak power from ADC and write WattsPeak
void CalculatePower()
{
  volatile int iAdcLocal, iAdcLocalPeak, iAdcLocalAvg;  // written by ISR, read in background
  
  // PwrFwd  Power forward
  noInterrupts();
  iAdcLocal     = PwrFwd.iAdc;
  iAdcLocalPeak = PwrFwd.iAdcPeak;
  iAdcLocalAvg  = PwrFwd.iAdcAvg;
  interrupts(); 

  PwrFwd.fVolts     = PwrFwd.fCal * iAdcLocal;
  PwrFwd.fVoltsPeak = PwrFwd.fCal * iAdcLocalPeak;
  PwrFwd.fVoltsAvg  = PwrFwd.fCal * iAdcLocalAvg;
  
  PwrFwd.fWatts = Watts(Settings.CouplerGainFwdDB, OffsetVoltFwd, PwrFwd.fVoltsAvg - OffsetVoltFwd); 
  
  // Mask interrupts to prevent reading while changing
  noInterrupts();
  iAdcLocal     = PwrRev.iAdc;
  iAdcLocalPeak = PwrRev.iAdcPeak;
  iAdcLocalAvg  = PwrRev.iAdcAvg; 
  interrupts();

  PwrRev.fVolts     = PwrRev.fCal * iAdcLocal;
  PwrRev.fVoltsPeak = PwrRev.fCal * iAdcLocalPeak;
  PwrRev.fVoltsAvg  = PwrRev.fCal * iAdcLocalAvg;
  
  PwrRev.fWatts = Watts(Settings.CouplerGainRevDB, OffsetVoltRev, PwrRev.fVoltsAvg - OffsetVoltRev); 
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
  CalculatePower();  // Peak ADC readings used to fill in Watts
  //SmoothDisplay(&PwrFwd, PwrFwd.fWatts);
  lcd.setCursor(0,0);
  lcd.print("Pwr Fwd ");
  sprintf(LcdString, "%4d", (int) PwrFwd.fWatts);
  lcd.print(LcdString);
  lcd.print("W   ");

  //SmoothDisplay(&PwrRev, PwrRev.fWatts);
  lcd.setCursor(0,1);
  lcd.print("Pwr Rev ");
  sprintf(LcdString, "%4d", (int) PwrRev.fWatts);
  lcd.print(LcdString);
  lcd.print("W     ");
} //Display Values


//*****************************************************************
// Drive the forward over power and reflected over power LEDs
// also add a "*" to the display
void DisplayLED()
{
  if( PwrFwd.fWatts > Settings.LimitFwd )
    { 
      digitalWrite(LimitPinFwd, HIGH); 
      lcd.setCursor(15,0);
      lcd.print("*");
    }
  else 
    { 
      digitalWrite(LimitPinFwd, LOW); 
      lcd.setCursor(15,0);
      lcd.print(" ");
    }

  if( PwrRev.fWatts > Settings.LimitRev )
    { 
      digitalWrite(LimitPinRev, HIGH); 
      lcd.setCursor(15,1);
      lcd.print("*");
    }
  else 
    { 
      digitalWrite(LimitPinRev, LOW); 
      lcd.setCursor(15,1);
      lcd.print(" ");
    }
}

//*******************************************************************
// Settings.MeterMode: RAW, Peak Hang, Peak Fast, Average
// Settings.MeterType: Power, SquareLaw, dBm
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

// Meter Characteristics
//    Integration Time Constant
//    Hold Time
//    Decay time Constant

// Settings.iAdcPeak has the peak voltage between display updates
// iAdcAvg = .1 * iAdc + .9 * iAdcAvg

// from: http://www.cypress.com/?docID=33575
// Vlp/Vin = 1 / (a + z^-1*(1-a))       Voltage Gain response
// Fo = Fs/(2*pi*a), a = Fs/(2*pi*Fo)   Frequency response
// s[n] = 1-(1-(1/a))^n                 step response
// a = N/2, for a moving average of N samples, IIR of a/2 works similarly
void DisplayMeter() 
{
  int dBm;
  int pwm;
  
  dBm = 10*log(PwrFwd.fWatts)+30.0;
  pwm = (int) dBm * 4;
  if (pwm <   0) pwm =   0;
  if (pwm > 255) pwm = 255;
  analogWrite(MeterPinFwd, pwm );

  //PwrRev and PwrFwd .fWatts, convert to dBm
  dBm = 10*log(PwrRev.fWatts)+30.0;
  pwm = (int) dBm * 4;
  if (pwm <   0) pwm =   0;
  if (pwm > 255) pwm = 255;
  analogWrite(MeterPinRev, pwm );
}

// monitor available, read command, output results
//-----------------------------------------
void serial()
{
  int cnt = 0;
  cnt = Serial.available();
  if (cnt > 0)
  {
    Serial.read();
    Serial.println(PwrFwd.iAdcPeak);
  }
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
  sprintf(LcdString, "%4d", (int) PwrFwd.fWatts);
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
  sprintf(LcdString, "%4d", (int) PwrRev.fWatts);
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

  // Check range, works for up/down, no button, EEPROM initialization
  if (Settings.BacklightLevel < BacklightLevelMin) 
    { Settings.BacklightLevel = BacklightLevelMin; }
  if (Settings.BacklightLevel > BacklightLevelMax) 
    { Settings.BacklightLevel = BacklightLevelMax; }

  analogWrite(BacklightPin, (Settings.BacklightLevel*255)/100);  // Set backlight brightness              

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
void MeterModeControl(int ButtonPressed, int ButtonPressTime)
{
   const char MeterModes[5][16] = {"RAW",
                                   "Peak Hang",
                                   "Peak Fast",
                                   "Average"};
  // handle corrupt or unitialized EERPOM
  const char  MeterModeIndexMin = 0;
  const char  MeterModeIndexMax = 3;

  if (ButtonPressTime == 0 )
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:     
        Settings.MeterModeIndex += 1;
        break;
      case DownButton:   
        Settings.MeterModeIndex -= 1;
        break;
    } // switch(ButtonPressed)
  } // if(ButtonPressTime == 0)

  // Check range, works for up/down, no button, EEPROM initialization
  if( Settings.MeterModeIndex < MeterModeIndexMin ) 
    { Settings.MeterModeIndex = MeterModeIndexMin; }      
  if( Settings.MeterModeIndex > MeterModeIndexMax ) 
    { Settings.MeterModeIndex = MeterModeIndexMax; }

  lcd.setCursor(0,0);
  lcd.print("Meter Dynamics  ");
  lcd.setCursor(0,1);
  lcd.print(MeterModes[Settings.MeterModeIndex]);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Controls what type of marking ar on the meter
//    Linear for regular volt/amp meter
//    Square Law for Bird Watt Meter type scale
void MeterTypeControl(int ButtonPressed, int ButtonPressTime)
{
  const char MeterTypes[5][16] = {"Linear",
                                  "Square Law",
                                  "dBm"};
  const int MeterTypeIndexMin = 0;
  const int MeterTypeIndexMax = 2;

  if (ButtonPressTime == 0)
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:     
        Settings.MeterTypeIndex += 1;
        break;
      case DownButton:   
        Settings.MeterTypeIndex -= 1;
        break;
    } // switch(ButtonPressed)
  } // if (ButtonPressTime == 0)  

  // Limit Range in all cases, even if no button pressed
  if( Settings.MeterTypeIndex > MeterTypeIndexMax ) 
    { Settings.MeterTypeIndex = MeterTypeIndexMin; }  // wrap around
  if( Settings.MeterTypeIndex < MeterTypeIndexMin )
    { Settings.MeterTypeIndex = MeterTypeIndexMax; }  // wrap around

  lcd.setCursor(0,0);
  lcd.print("Meter Type      ");
  lcd.setCursor(0,1);
  lcd.print(MeterTypes[Settings.MeterTypeIndex]);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Meter Scale
void MeterScaleFwdControl(int ButtonPressed, int ButtonPressTime)
{
  const int MeterScaleMin = 2;
  const int MeterScaleMax = 5000;  

  // Limit range to between Min and Max
  if( Settings.MeterScaleFwd < MeterScaleMin) 
    { Settings.MeterScaleFwd = MeterScaleMin; }   
  if( Settings.MeterScaleFwd > MeterScaleMax)
    { Settings.MeterScaleFwd = MeterScaleMax; }   
  
  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      Settings.MeterScaleFwd += 1 * .001 * ButtonPressTime;
      break;
    case DownButton:            
      Settings.MeterScaleFwd -= 1 * .001 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Meter Fwd Max ");
  lcd.setCursor(0,1);
  lcd.print( (int) Settings.MeterScaleFwd );
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Meter Scale
void MeterScaleRevControl(int ButtonPressed, int ButtonPressTime)
{
  const int MeterScaleMin = 2;
  const int MeterScaleMax = 5000;  
  
  // Limit range to between Min and Max
  if( Settings.MeterScaleRev < MeterScaleMin ) 
    { Settings.MeterScaleRev = MeterScaleMin; }   
  if( Settings.MeterScaleRev > MeterScaleMax )
    { Settings.MeterScaleRev = MeterScaleMax; }   

  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:
        Settings.MeterScaleRev += 1 * .002 * ButtonPressTime;
      break;
    case DownButton:            
        Settings.MeterScaleRev -= 1 * .002 * ButtonPressTime;
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Meter Rev Max ");
  lcd.setCursor(0,1);
  lcd.print((int) Settings.MeterScaleRev );
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Meter Averaging Time Constant
void MeterAvgTcControl(int ButtonPressed, int ButtonPressTime)
{
  const int MeterAvgTcMin = 1;
  const int MeterAvgTcMax = 2000;  
  
  // Limit range to between Min and Max
  if( Settings.MeterAvgTc < MeterAvgTcMin ) 
    { Settings.MeterAvgTc = MeterAvgTcMin; }   
  if( Settings.MeterAvgTc > MeterAvgTcMax )
    { Settings.MeterAvgTc = MeterAvgTcMax; }   
  
  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      if(ButtonPressTime == 0)
        { Settings.MeterAvgTc += 1; }
      else
        { Settings.MeterAvgTc += 1 * .001 * ButtonPressTime; }
      break;
    case DownButton:            
      if(ButtonPressTime == 0)
        { Settings.MeterAvgTc -= 1; }
      else
        { Settings.MeterAvgTc -= 1 * .001 * ButtonPressTime; }
      break;
  } // switch(ButtonPressed)

  // compute iAdcIIR coefficient, 
  // iAdcAvg = (1-iAdcIIR) * iAdcAvg + iAdcIIR * iAdc
  // MeterAvgTc and TimerBetweenInterrupts in msec
  PwrFwd.fAdcIIR = 1.0/(Settings.MeterAvgTc/TimeBetweenInterrupts);
  PwrRev.fAdcIIR = PwrFwd.fAdcIIR;    // Same values
  
  
  lcd.setCursor(0,0);
  lcd.print("Meter Avg ms   ");
  lcd.setCursor(0,1);
  lcd.print( (int) Settings.MeterAvgTc );
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// Define the Meter Decay Time Constand
//
void MeterDecayTcControl(int ButtonPressed, int ButtonPressTime)
{
  const int MeterDecayTcMin = 1;
  const int MeterDecayTcMax = 2000;  
  
  // Limit range to between Min and Max
  if( Settings.MeterDecayTc < MeterDecayTcMin ) 
    { Settings.MeterDecayTc = MeterDecayTcMin; }   
  if( Settings.MeterDecayTc > MeterDecayTcMax )
    { Settings.MeterDecayTc = MeterDecayTcMax; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Button still pressed
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:  
      if(ButtonPressTime == 0)
        { Settings.MeterDecayTc += 1; }
      else
        { Settings.MeterDecayTc += 1 * .001 * ButtonPressTime; }
      break;
    case DownButton:            
      if(ButtonPressTime == 0)
        { Settings.MeterDecayTc -= 1; }
      else
        { Settings.MeterDecayTc -= 1 * .001 * ButtonPressTime; }
      break;
  } // switch(ButtonPressed)

  lcd.setCursor(0,0);
  lcd.print("Meter Decay ms   ");
  lcd.setCursor(0,1);
  lcd.print( (int) Settings.MeterDecayTc );
  lcd.print("                ");  // finish out the line with blanks  
}

//***********************************************************************
void ProcessPowerDisplay()
{
  DisplayPower();      // LCD numbers, P.fwd and P.rev
  DisplayMeter();      // PWM to meter or bar graph
  DisplayLED();     
}

//******************************************************************************
// state machine, Entered at timed intervals
// switch on Control vs Power Display mode
// Select button change between Control and Power mode
// Left and Right buttons change the control mode
// Up and Down handled in the Control routines
// then call ProcessControlMode(ModeIndex) which branches to control routines
void DisplayMachine()
{ 
  // List of all states
  enum state_t 
  { PowerMode, ControlMode };
  static state_t DisplayState = PowerMode;
  
  // define type for a pointer to a control function
  typedef void (*fControlPtr)(int, int);
  // build an array of control routine pointers.
  fControlPtr fControl[11] = 
  { 
           FwdCalControl,
           RevCalControl,
        MeterModeControl,
        BacklightControl,
         FwdLimitControl,
         RevLimitControl,
        MeterTypeControl,
    MeterScaleFwdControl,
    MeterScaleRevControl,
       MeterAvgTcControl,
     MeterDecayTcControl
  };
  
  const char           FwdMode =  0;
  const char           RevMode =  1;
  const char         MeterMode =  2;
  const char     BacklightMode =  3;
  const char      FwdLimitMode =  4;
  const char      RevLimitMode =  5;
  const char     MeterTypeMode =  6;
  const char MeterScaleFwdMode =  7;
  const char MeterScaleRevMode =  8;
  const char    MeterAvgTcMode =  9;
  const char  MeterDecayTcMode = 10;
  
  static int ModeIndex    = 0;
   const int ModeIndexMin = 0;
   const int ModeIndexMax = 10;

  const int ControlTimeout = 5000; // timeout in 5 seconds
  ReadButton();  // sets value of ButtonPressed and ButtonPressTime
  switch (DisplayState)  // Control mode or Power Mode
  {
    case ControlMode:  
      switch (ButtonPressed)
      {   
        case NoButton:
          if (ButtonPressTime == ControlTimeout)
          {
            // Revert to Power display, erase control stuff from screen
            DisplayState = PowerMode;
          }
          break;  // NoButton

        case SelectButton:
          // arrive here in two conditions
          //    Button just pressed, time=0, change state
          if (ButtonPressTime == 0)   // on leading edge of button press
          {
            // user selects end, write to EEPROM
            eeprom_write_block((const void*)&Settings, (void*)0, sizeof(Settings));
            DisplayState = PowerMode;
          }
          break;  // SelectButton
       
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

        case UpButton:
        case DownButton:    
          // handled in ProcessCurrentMode()    
          break; 
      }  // switch(ButtonPressed)
      
      // Call Control routine using the function point array      
      (*fControl[ModeIndex])(ButtonPressed, ButtonPressTime);
      
      break;  // ControlMode

    case PowerMode:
      switch (ButtonPressed)
      {
        // all buttons but Select just continue to update display
        case LeftButton:   
        case RightButton:   
        case UpButton:      
        case DownButton: 
        case NoButton:
          ProcessPowerDisplay();
          break;

        case SelectButton:
          // return to power mode
          if (ButtonPressTime == 0)
          {
            DisplayState = ControlMode;
            
            // Call Control routine using the function point array
            (*fControl[ModeIndex])(ButtonPressed, ButtonPressTime);        
          }
          else
          {
            // Here if holding down Select after pressing in Control mode
            ProcessPowerDisplay();
          }
          break;
      } // switch(ButtonPressed)
      break; // PowerMode
    default: 
      { }
    } // switch (DisplayState)
  } // StateMachine()

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

//*******************************************************************************
// Configure the pins, set outputs to standby
// Enable interrupts at the end
void setup() 
{
  // used to set floating point ADC calibration value
  const int   AdcMaxCount = 1023;
  const float AdcMaxVolts = 5.0;
  
  // Setup pin modes
  pinMode( MeterPinFwd, OUTPUT);
  pinMode( MeterPinRev, OUTPUT);
  pinMode( LimitPinFwd, OUTPUT);
  pinMode( LimitPinRev, OUTPUT);
  pinMode(BacklightPin, OUTPUT);

  // read and fill in the calibration Settings values
  eeprom_read_block( (void*)&Settings, (void*)0, sizeof(Settings) );

  analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness

  // set up the LCD's number of columns and rows: 
  lcd.clear();
  lcd.begin(16, 2);

  lcd.setCursor(0,0);
  lcd.print("  Power Meter   ");
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
  
  OffsetVoltFwd = PwrFwd.fVolts;
  OffsetVoltRev = PwrRev.fVolts;

  // for debug purposes
  OffsetVoltFwd = 0.6;
  OffsetVoltRev = 0.6;
 
  // The serial port 
  //Serial.begin(9600);
  
  Firmata.setFirmwareVersion(0,1);
  Firmata.begin();
  
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
    // Measure time to process data and display, about 14-19 msec
    //DisplayTime = micros() - DisplayTime;
    //lcd.setCursor(0,0);
    //lcd.print(IsrTime);
    //lcd.setCursor(0,1);
    //lcd.print(DisplayTime/1000.0, 1);
    //lcd.print(" ");
    //delay(20);
    //serial();             // put info on serial port
  }
}
