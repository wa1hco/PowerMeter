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

// Function Prototypes
// Prototpye for external Watts function written by Matlab
float Watts( float CouplerGainFwdDB, float Vol, float V );

void     FwdCalControl(int, int);
void     RevCalControl(int, int);
void  MeterModeControl(int, int);
void  BacklightControl(int, int);
void   FwdLimitControl(int, int);
void   RevLimitControl(int, int);
void  MeterTypeControl(int, int);
void MeterScaleControl(int, int);

typedef void (*fControlPtr)(int, int);

fControlPtr fControl[8] = 
{ 
      FwdCalControl,
      RevCalControl,
   MeterModeControl,
   BacklightControl,
    FwdLimitControl,
    RevLimitControl,
   MeterTypeControl,
  MeterScaleControl 
};

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
  volatile int   iADC;        // ADC output in count from 0 to AdcMaxCount
  volatile int   iADCPeak;    // Peak value, holds and decays
  volatile int   PeakTimer;   // Time since peak updated
           float fVolts;      // fCal * iADC
           float fVoltsPeak;  // fCal * iADCPeak
           float fWatts;      // Power after curve fit
           float fCal;        // Multipy by iADC reading to give fVolts
           float DisplayNow;  // The value on the display at the moment
           int   DisplayTime; // Time elaspsed since last display change
};

// Define the analog inputs
struct analog_t PwrRev;    // Power, Reverse from sampling
struct analog_t PwrFwd;    // Power, Forward from sampling

// Globals for button press processing
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

const int     TimeBetweenInterrupts =  2;  // msec, time calls to ISR
const int TimeBetweenDisplayUpdates = 50;  // msec, time calls to Display and Button update

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
  char  MeterModeIndex;    //  
  int   BacklightLevel;    // 0 to 255
  int   LimitRev;          // 0 to 300 Watts
  int   LimitFwd;          // 0 to 2000 Watts
  int   MeterTypeIndex;     // {Linear, Square Law}
  int   MeterScaleIndex;    // {1, 2, 5}
} Settings;  

char LcdString[17];  // for sprintf, include null term, needed ???

//**************************************************************************
// ADC Peak Processing
// if greater, Update iADCPeak, and start timer 
// delay tHold, then start decay of value in iADCPeak
void ADC_Peak(struct analog_t *sig)
{
  // Meter hang time
  const int tHold = 500;  // msec
  
  if(sig->iADC > sig->iADCPeak) // if new peak
  {
    sig->iADCPeak = sig->iADC;  // save new peak
    sig->PeakTimer = 0;       // restart peak timer
  } // new peak
  else // not new peak
  {
    if(sig->PeakTimer < tHold) // peak hold not elapsed
    {  
      sig->PeakTimer += TimeBetweenInterrupts; // add elapsed time, leave peak alone
    }
    else // peak hold elapsed, exponential decay
    {
      // Exponential decay
      sig->iADCPeak *= .998;
    }
  } // not new peak
} // ADC_Peak

// Entered at TimeIncement intervals, 
// called from interrupt context
// read both ADC input close together in time
// then do the peak processing
// ADC reads occur at 2 ms, 500 Hz
// Display processing reads values in iAdc and iAdcPeak at 50 ms, 20 Hz
//-----------------------------------------
void UpdateAnalogInputs() 
{
  // read from the ADC twice close together
  PwrFwd.iADC = analogRead(PwrFwd.Pin); // read the raw ADC value 
  PwrRev.iADC = analogRead(PwrRev.Pin); // read the raw ADC value 
  
  // test input
  PwrFwd. iADC = 0x3ff & (int) millis() >> 3 ;
  PwrRev. iADC = 0x3ff & (int) millis() >> 3 ;

  // ADCs read at 500 Hz, Display updates at 20 Hz, show Peaks
  ADC_Peak(&PwrFwd);  // Update PwrFwd data structure with peak hold
  ADC_Peak(&PwrRev);  // Update Pr data structure with peak hold
} // UpdateAnalogInputs()

//********************************************************************************
// Enter at TimeInterval = 2 ms
boolean DisplayFlag = false;
int IsrTime = 0;  // debugging variable
//-------------------------------------------
void TimedService() 
{
  static int DisplayTimeCounter = 0;
  // debug
  IsrTime = micros(); 
  UpdateAnalogInputs();
  // set a flag periodically to run display update
  DisplayTimeCounter += TimeBetweenInterrupts;
  if (DisplayTimeCounter > TimeBetweenDisplayUpdates) 
  {
    DisplayTimeCounter = 0;
    DisplayFlag = true;
  }
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
  volatile int iAdcLocal, iAdcLocalPeak;  // written by ISR, read in background
  
  // PwrFwd  Power forward
  noInterrupts();
  iAdcLocal     = PwrFwd.iADC;
  iAdcLocalPeak = PwrFwd.iADCPeak;
  interrupts(); 
  PwrFwd.fVolts     = PwrFwd.fCal * iAdcLocal;
  PwrFwd.fVoltsPeak = PwrFwd.fCal * iAdcLocalPeak;
  
  PwrFwd.fWatts = Watts(Settings.CouplerGainFwdDB, OffsetVoltFwd, PwrFwd.fVoltsPeak - OffsetVoltFwd); 
  
  // Mask interrupts to prevent reading while changing
  noInterrupts();
  iAdcLocal     = PwrRev.iADC;
  iAdcLocalPeak = PwrRev.iADCPeak;
  interrupts();
  PwrRev.fVolts     = PwrRev.fCal * iAdcLocal;
  PwrRev.fVoltsPeak = PwrRev.fCal * iAdcLocalPeak;
  
  PwrRev.fWatts = Watts(Settings.CouplerGainRevDB, OffsetVoltRev, PwrRev.fVoltsPeak - OffsetVoltRev); 
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
  PwrFwd.fWatts = Watts(Settings.CouplerGainFwdDB, OffsetVoltFwd, PwrFwd.fVoltsPeak - OffsetVoltFwd); 
  //PwrFwd.fWatts  =  PwrFwd.fCal * (float) iAdcLocal-OffsetVoltFwd;
  //SmoothDisplay(&PwrFwd, PwrFwd.fWatts);
  lcd.setCursor(0,0);
  lcd.print("Pwr Fwd ");
  sprintf(LcdString, "%4d", (int) PwrFwd.fWatts);
  lcd.print(LcdString);
  lcd.print("W   ");

  PwrRev.fWatts = Watts(Settings.CouplerGainRevDB, OffsetVoltRev, PwrRev.fVoltsPeak - OffsetVoltRev); 
  //SmoothDisplay(&PwrRev, PwrRev.fWatts);
  lcd.setCursor(0,1);
  lcd.print("Pwr Rev ");
  sprintf(LcdString, "%4d", (int) PwrRev.fWatts);
  lcd.print(LcdString);
  lcd.print("W     ");
} //Display Values


//*****************************************************************
// drive the forward over power and reflected over power LEDs
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
// Settings.MeterMode
// Settings.MeterType
// Settings.MeterScale
// Called at display update time
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
    Serial.println(PwrFwd.iADCPeak);
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
  const float CouplerGainFwdDBMin =   -60;
  const float CouplerGainFwdDBMax =   -10;
  if(Settings.CouplerGainFwdDB < CouplerGainFwdDBMin || Settings.CouplerGainFwdDB > CouplerGainFwdDBMax)
    { Settings.CouplerGainFwdDB = (CouplerGainFwdDBMin + CouplerGainFwdDBMax)/2; }
  switch (ButtonPressed)
  {
    case SelectButton:      // Still pressed from entering Control mode
    case LeftButton:        // Still pressed
    case RightButton:       // Still pressed
      break;
    case UpButton:
      Settings.CouplerGainFwdDB += 0.1 * .001 * ButtonPressTime;
      if (Settings.CouplerGainFwdDB > CouplerGainFwdDBMax)
        { Settings.CouplerGainFwdDB = CouplerGainFwdDBMax; }
      break;
    case DownButton:          
      Settings.CouplerGainFwdDB -= 0.1 * .001 * ButtonPressTime;
      if (Settings.CouplerGainFwdDB < CouplerGainFwdDBMin)
        { Settings.CouplerGainFwdDB = CouplerGainFwdDBMin; }
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
  const float CouplerGainRevDBMin =   -60;
  const float CouplerGainRevDBMax =   -10;
  if(Settings.CouplerGainRevDB < CouplerGainRevDBMin || Settings.CouplerGainRevDB > CouplerGainRevDBMax)
    { Settings.CouplerGainRevDB = (CouplerGainRevDBMin + CouplerGainRevDBMax)/2; }

  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:            
      Settings.CouplerGainRevDB += 0.1 * .001 * ButtonPressTime;
      if (Settings.CouplerGainRevDB > CouplerGainRevDBMax)
        { Settings.CouplerGainRevDB = CouplerGainRevDBMax; }
      break;
    case DownButton:
      Settings.CouplerGainRevDB -= 0.1 * .001 * ButtonPressTime;
      if (Settings.CouplerGainRevDB < CouplerGainRevDBMin)
        { Settings.CouplerGainRevDB = CouplerGainRevDBMin; }     
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
void MeterModeControl(int ButtonPressed, int ButtonPressTime)
{
   const char MeterModes[5][16] = {"RAW",
                                  "Peak Hang",
                                  "Peak Fast",
                                  "Average"};
  // handle corrupt or unitialized EERPOM
  const char  MeterModeIndexMin   =     0;
  const char  MeterModeIndexMax   =     3;
  if(Settings.MeterModeIndex < MeterModeIndexMin || Settings.MeterModeIndex > MeterModeIndexMax)
    { Settings.MeterModeIndex = (MeterModeIndexMin + MeterModeIndexMax)/2; }
  if (ButtonPressTime == 0 )
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:     
        if(Settings.MeterModeIndex < MeterModeIndexMax) 
          { Settings.MeterTypeIndex += 1; }
        else if(Settings.MeterModeIndex == MeterModeIndexMax)
          { Settings.MeterModeIndex = MeterModeIndexMin; }
        break;
      case DownButton:   
        if(Settings.MeterModeIndex > MeterModeIndexMin) 
          { Settings.MeterModeIndex -= 1; }      
        else if(Settings.MeterModeIndex == MeterModeIndexMin)
          { Settings.MeterModeIndex = MeterModeIndexMax; }
        break;
  } // switch(ButtonPressed)
  }
  lcd.setCursor(0,0);
  lcd.print("Meter Dynamics  ");
  lcd.setCursor(0,1);
  lcd.print(MeterModes[Settings.MeterModeIndex]);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
// called in Control Mode for any button press
void BacklightControl(int ButtonPressed, int ButtonPressTime)
{
  // handle corrupt or unitialized EERPOM
  const int   BacklightLevelMin   =     0;
  const int   BacklightLevelMax   =   255;
  if(Settings.BacklightLevel < BacklightLevelMin || Settings.BacklightLevel > BacklightLevelMax)
    { Settings.BacklightLevel = (BacklightLevelMin + BacklightLevelMax)/2; }
  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:
      Settings.BacklightLevel += 1 * .001 * ButtonPressTime;
      if (Settings.BacklightLevel > BacklightLevelMax) 
        { Settings.BacklightLevel = BacklightLevelMax; }
      analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness              
      break;
    case DownButton:
      Settings.BacklightLevel -= 1 * .001 * ButtonPressTime;
      if (Settings.BacklightLevel < BacklightLevelMin) 
        { Settings.BacklightLevel = BacklightLevelMin; }
      analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness              
      break;
  } // switch(ButtonPressed)
  lcd.setCursor(0,0);
  lcd.print("Backlight ");
  lcd.print((100 * Settings.BacklightLevel) / BacklightLevelMax);
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
  const int   LimitFwdMin         =    10;
  const int   LimitFwdMax         =  1500;
  if(Settings.LimitFwd < LimitFwdMin || Settings.LimitFwd > LimitFwdMax)
    { Settings.LimitFwd = (LimitFwdMin + LimitFwdMax)/2; }

  switch (ButtonPressed)
  {
    case SelectButton:      // Still pressed from entering Control mode
    case LeftButton:        // Still pressed
    case RightButton:       // Still pressed
      break;
    case UpButton:
      Settings.LimitFwd += 1 * .001 * ButtonPressTime;
      if (Settings.LimitFwd > LimitFwdMax)
        { Settings.LimitFwd = LimitFwdMax; }
      break;
    case DownButton:          
      Settings.LimitFwd -= 1 * .001 * ButtonPressTime;
      if (Settings.LimitFwd < LimitFwdMin)
        { Settings.LimitFwd = LimitFwdMin; }
      break;
  } // switch(ButtonPressed)
  lcd.setCursor(0,0);
  lcd.print("Fwd Limit ");
  lcd.print(Settings.LimitFwd);
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
  const int   LimitRevMin         =    10;
  const int   LimitRevMax         =   300;
  if(Settings.LimitRev < LimitRevMin || Settings.LimitRev > LimitRevMax)
    { Settings.LimitRev = (LimitRevMin + LimitRevMax)/2; } 

  switch (ButtonPressed)
  {
    case SelectButton:      // pressed to exit Select mode
    case LeftButton:        // after changing to this mode from Left
    case RightButton:       // after changing to this mode from Right
      break;
    case UpButton:            
      Settings.LimitRev += 1 * .001 * ButtonPressTime;
      if (Settings.LimitRev > LimitRevMax)
        { Settings.LimitRev = LimitRevMax; }
      break;
    case DownButton:
      Settings.LimitRev -= 1 * .001 * ButtonPressTime;
      if (Settings.LimitRev < LimitRevMin)
        { Settings.LimitRev = LimitRevMin; }     
      break;
  } // switch(ButtonPressed)
  lcd.setCursor(0,0);
  lcd.print("Rev Limit ");
  lcd.print(Settings.LimitRev);
  lcd.print("W        ");
  lcd.setCursor(0,1);
  lcd.print("                ");
}

//******************************************************************************
// Controls what type of marking ar on the meter
//    Linear for regular volt/amp meter
//    Square Law for Bird Watt Meter type scale
void MeterTypeControl(int ButtonPressed, int ButtonPressTime)
{
  const char MeterTypes[5][16] = {"Linear",
                                  "Square Law"};
  const int MeterTypeIndexMin = 0;
  const int MeterTypeIndexMax = 1;

  // handle corrupt or unitialized EERPOM  
  if(Settings.MeterTypeIndex < MeterTypeIndexMin || Settings.MeterTypeIndex > MeterTypeIndexMax)
    { Settings.MeterTypeIndex = MeterTypeIndexMin; }
  
  if (ButtonPressTime == 0)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:     
        if(Settings.MeterTypeIndex < MeterTypeIndexMax) 
          { Settings.MeterTypeIndex += 1; }
        else if(Settings.MeterTypeIndex == MeterTypeIndexMax)
          { Settings.MeterTypeIndex = MeterTypeIndexMin; }
        break;
      case DownButton:   
        if(Settings.MeterTypeIndex > MeterTypeIndexMin) 
          { Settings.MeterTypeIndex -= 1; }      
        else if(Settings.MeterTypeIndex == MeterTypeIndexMin)
          { Settings.MeterTypeIndex = MeterTypeIndexMax; }
        break;
    } // switch(ButtonPressed)
  } // if (!NoButton)  
  lcd.setCursor(0,0);
  lcd.print("Meter Type      ");
  lcd.setCursor(0,1);
  lcd.print(MeterTypes[Settings.MeterTypeIndex]);
  lcd.print("                ");  // finish out the line with blanks  
}

//******************************************************************************
void MeterScaleControl(int ButtonPressed, int ButtonPressTime)
{
  const char MeterScales[12][16] = {"  10", "  20", "  25", "  50",
                                    " 100", " 200", " 250", " 500",
                                    "1000", "2000", "2500", "5000"};
  const int MeterScaleIndexMin = 0;
  const int MeterScaleIndexMax = 11;  
  
  if(Settings.MeterScaleIndex < MeterScaleIndexMin || Settings.MeterScaleIndex > MeterScaleIndexMax)
    { Settings.MeterScaleIndex = MeterScaleIndexMin; }
  
  // Process leading edge of button presses, no press and hold  
  if (ButtonPressed != NoButton && ButtonPressTime == 0) 
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:  
        if(Settings.MeterScaleIndex < MeterScaleIndexMax) 
          { Settings.MeterScaleIndex += 1; }   
        else if(Settings.MeterScaleIndex == MeterScaleIndexMax)
          { Settings.MeterScaleIndex = MeterScaleIndexMin;}
        break;
      case DownButton:            
        if(Settings.MeterScaleIndex > MeterScaleIndexMin) 
          { Settings.MeterScaleIndex -= 1; }  
        else if(Settings.MeterScaleIndex == MeterScaleIndexMin)
          { Settings.MeterScaleIndex = MeterScaleIndexMax;}
        break;
    } // switch(ButtonPressed)
  } // if (!NoButton)  
  lcd.setCursor(0,0);
  lcd.print("Meter Scale Max ");
  lcd.setCursor(0,1);
  lcd.print(MeterScales[Settings.MeterScaleIndex]);
  lcd.print("                ");  // finish out the line with blanks  
}

//********************************************************************
// called on entering new control mode
// called on every display update when Up or Down button is pressed
// called with select button still pressed
void ProcessCurrentMode(int ModeIndex)
{
  switch (ModeIndex) // FwdMode=0, RevMode=1, MeterMode=2, BacklightMode=3
    {
      case 0:  // Fwd Calibration
        FwdCalControl(ButtonPressed, ButtonPressTime);
        break;
      case 1:  // Rev Calibration
        RevCalControl(ButtonPressed, ButtonPressTime);
        break;
      case 2:  // Meter Mode
        MeterModeControl(ButtonPressed, ButtonPressTime);
        break;
      case 3:  // Backlight Level
        BacklightControl(ButtonPressed, ButtonPressTime);
        break;
      case 4:  // Fwd Limit
        FwdLimitControl(ButtonPressed, ButtonPressTime);
        break;
      case 5:  // Rev Limit 
        RevLimitControl(ButtonPressed, ButtonPressTime);
        break;
      case 6:  // 
        MeterTypeControl(ButtonPressed, ButtonPressTime);
        break;
      case 7:  // 
        MeterScaleControl(ButtonPressed, ButtonPressTime);
        break;
    } // switch(ModeIndex
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

  const char FwdMode         = 0;
  const char RevMode         = 1;
  const char MeterMode       = 2;
  const char BacklightMode   = 3;
  const char FwdLimitMode    = 4;
  const char RevLimitMode    = 5;
  const char MeterTypeMode   = 6;
  const char MeterScaleMode  = 7;
  
  static int ModeIndex       = 0;
   const int ModeIndexMin    = 0;
   const int ModeIndexMax    = 7;

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
      ProcessCurrentMode(ModeIndex);
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
            ProcessCurrentMode(ModeIndex); // starts mode display on Select
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
  eeprom_read_block((void*)&Settings, (void*)0, sizeof(Settings));  

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
  PwrFwd.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iADC to float Volts
  PwrRev.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iADC to float Volts

  
  // TODO: Very wrong as written
  // Idea: Amp has limits on Pf and Pr in Watts, convert back to ADC for fast compare
  // Limits in floating point
  //PwrFwd.fWattsMax  = 1250.0;  // Watts
  //PwrRev.fWattsMax  =  100.0;  // Watts
   
  //PwrFwd.iADCMax = PwrFwd.fVoltsMax / PwrFwd.fCal; //TODO: not right
  //PwrRev.iADCMax = PwrRev.fVoltsMax / PwrRev.fCal; //TODO: not right
  
  // convert gain of coupler from dB to linear
  //CouplerGainFwdLinear = pow(10, Settings.CouplerGainFwdDB/10);
  //CouplerGainRevLinear = pow(10, Settings.CouplerGainRevDB/10);
  
  // Read Forward and Reverse ADC values, assuming no RF to set Vol
  PwrFwd.fVolts = PwrFwd.fCal * analogRead(PwrFwd.iADC);
  PwrRev.fVolts = PwrRev.fCal * analogRead(PwrRev.iADC);
  
  OffsetVoltFwd = PwrFwd.fVolts;
  OffsetVoltRev = PwrRev.fVolts;

  // for debug purposes
  OffsetVoltFwd = 0.6;
  OffsetVoltRev = 0.6;
 
  // The serial port 
  //Serial.begin(9600);
  
  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TimeBetweenInterrupts, TimedService); // 2ms period
  // interrupts enabled after this point
  MsTimer2::start(); 
} // setup


//*****************************************************************
// Main Loop, just for display stuff
// Arduino falls into this after setup()
// This loop gets interrupted periodically
// All timed stuff occurs in ISR
//--------------------------------------------------------------
void loop() 
{
  // watch for signal from interrupt handler
  // DisplayFlag set in interrupt handler Timebetween 
  if (DisplayFlag == true) 
  {
    DisplayFlag = false;  // clear the flag
    DisplayMachine();     // state machine based on ButtonPressed
    serial();             // put info on serial port
  }
}
