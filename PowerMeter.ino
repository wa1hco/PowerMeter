/*
 * Power Meter for Dual Directional Coupler with LTC5507
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
//  0  Pf 2000W
//  1  Pr 2000W  

//  Control Display Layout 
//     0123456789012345 
//  0  Pf 2000W
//  1  Pr 2000W  


// include the library code:
#include           <math.h>
#include  <LiquidCrystal.h>
#include  <avr/interrupt.h>
#include       <MsTimer2.h>
#include     <avr/eeprom.h>

// Function Prototypes
// Prototpye for external Watts function written by Matlab
float Watts( float CouplerGainFwdDB, float Vol, float V );

// initialize
// LiquidCrystal(rs, enable, d4, d5, d6, d7) 
// LCD 1602 also controls backlight on Pin 10
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);  // LCD1602

// Available Digital Pins Map
// Digital:    0 1 2 3 4 5 6 7 8 9 10 11 12 13
// LCD:                4 5 6 7 8 9 10              // Backlight on 10           
// PWM:              3   5 6    9 10 11
// !used&PWM         3                11
// LED:        0 1
// !used&!PWM      2                     12, 13

// Digital Pins
int     MeterPinFwd =  3;  // PWM output to Forward meter
int     MeterPinRev = 11;  // PWM output to Reverse meter
int     RevLimitPin =  1;  // LED Output on high reflected power
int     FwdLimitPin =  0;  // LED Output on high peak power
int    BacklightPin = 10;  // high for backlight on

const int   AdcMaxCount = 1023;
const float AdcMaxVolts = 5.0;

// Meter hang time
const int tHold = 250;  // msec

// All the analog inputs have the same structure
// Not all analog inputs use all values
// volatile because shared between interrupt and background display context
struct analog_t 
{
    volatile int Pin;         // Arduino pin number
    volatile int iADC;        // ADC output in count from 0 to AdcMaxCount
    volatile int iADCPeak;    // Peak value, holds and decays
    volatile int iADCMin;     // Fault if raw below min
    volatile int iADCMax;     // Fault if raw above max
  volatile float fVal;        // fCal * iADC
  volatile float fValPeak;    // fCal * iADCPeak
  volatile float fValMin;     // fCal * iADCMin
  volatile float fValMax;     // fCal * iADCMax
    volatile int PeakTimer;   // Time since peak updated
    volatile int TimePerRead; // Time between ADC reads
  volatile float fCal;        // Multipy by iADC reading to give fVal
  volatile float DisplayNow;  // The value on the display at the moment
    volatile int DisplayTime; // Time elaspsed since last display change
};

// List of all states
enum state_t 
{
  PowerMode,
  ControlMode
} DisplayState;

enum button_t
{
  NoButton,
  SelectButton,
  LeftButton, 
  RightButton, 
  UpButton, 
  DownButton,
} ButtonPressed;


// Define the analog inputs
struct analog_t Pr;    // Power, Reverse from sampling
struct analog_t Pf;    // Power, Forward from sampling
struct analog_t ButtonV;   // Pushbutton analog value

// Message Buffer
volatile char Message[11] = "wa1hco    ";

const int TimeBetweenInterrupts = 2;  // milliseconds

// define the LTC5507 offsets, measured at setup, used in curve fit
float Vol_forward;
float Vol_reverse;

// Control mode starts with Select, Display mode

static int ButtonPressTime = 0;
const int ButtonPressTimeMax = 5000;

// Nonvolatile configuration settings
// read on startup
struct settings_t
{
  float CouplerGainFwdDB;  // -10 to -60 dB
  float CouplerGainRevDB;  // -10 to -60 dB
  char  MeterModeIndex;    //  
  int   BacklightLevel;    // 0 to 255
  int   LimitRev;       // 0 to 300 Watts
  int   LimitFwd;       // 0 to 2000 Watts
} Settings;  

char ModeNames[6][17] = {"Fwd Cal ",
                         "Rev Cal ",               
                         "Meter ",
                         "Backlight ",
                         "Fwd Limit",
                         "Rev Limit"};

const char FwdMode       = 0;
const char RevMode       = 1;
const char MeterMode     = 2;
const char BacklightMode = 3;
const char FwdLimitMode  = 4;
const char RevLimitMode  = 5;

int ModeIndex = 0;
const int   ModeIndexMin        =  0;
const int   ModeIndexMax        =  5;

char MeterModes[5][16] = {"RAW       ",
                          "Peak Hang ",
                          "Peak Fast ",
                          "Average   "};

// Limits on nonvolatile settings
const float CouplerGainFwdDBMin = -60;
const float CouplerGainFwdDBMax = -10;
const float CouplerGainRevDBMin = -60;
const float CouplerGainRevDBMax = -10;
const char  MeterModeIndexMin   =  0;
const char  MeterModeIndexMax   =  3;
const int   BacklightLevelMin   =  0;
const int   BacklightLevelMax   =  255;
const int   LimitFwdMin          =  10;
const int   LimitFwdMax          =  1500;
const int   LimitRevMin          =  10;
const int   LimitRevMax          =  300;

float CouplerGainFwdLinear;
float CouplerGainRevLinear;
char CouplerGainStr[10];

//**************************************************************************

int adc_key_val[5] ={50, 200, 400, 600, 800 };
int NUM_KEYS = 5;
int adc_key_in;
int key=-1;
int oldkey=-1;

//**************************************************************************
// Set any out of range nonvolatile value to mid point
// Handles blank or corrupted EEPROM
void CheckNonvolatile()
{
 if(Settings.CouplerGainFwdDB < CouplerGainFwdDBMin || Settings.CouplerGainFwdDB > CouplerGainFwdDBMax)
   { Settings.CouplerGainFwdDB = (CouplerGainFwdDBMin + CouplerGainFwdDBMax)/2; }
 if(Settings.CouplerGainRevDB < CouplerGainRevDBMin || Settings.CouplerGainRevDB > CouplerGainRevDBMax)
   { Settings.CouplerGainRevDB = (CouplerGainRevDBMin + CouplerGainRevDBMax)/2; }
 if(Settings.MeterModeIndex < MeterModeIndexMin || Settings.MeterModeIndex > MeterModeIndexMax)
   { Settings.MeterModeIndex = (MeterModeIndexMin + MeterModeIndexMax)/2; }
 if(Settings.BacklightLevel < BacklightLevelMin || Settings.BacklightLevel > BacklightLevelMax)
   { Settings.BacklightLevel = (BacklightLevelMin + BacklightLevelMax)/2; }
 if(Settings.LimitFwd < LimitFwdMin || Settings.LimitFwd > LimitFwdMax)
   { Settings.LimitFwd = (LimitFwdMin + LimitFwdMax)/2; }
 if(Settings.LimitRev < LimitRevMin || Settings.LimitRev > LimitRevMax)
   { Settings.LimitRev = (LimitRevMin + LimitRevMax)/2; } 
}

//**************************************************************************
// ADC Read and Peak Hander function
// read analog value into iADC
void ADC_Read(struct analog_t *sig) 
{
  sig->iADC = analogRead(sig->Pin); // read the raw ADC value 
} // ADC_Read

//**************************************************************************
// ADC Peak Processing
// if greater, Update iADCPeak, and start timer 
// delay tHold, then start decay of value in iADCPeak
// TODO: subtracting TimePerRead from Voltage is a kludge
void ADC_Peak(struct analog_t *sig)
{
  if(sig->iADC > sig->iADCPeak) // if new peak
  {
    sig->iADCPeak = sig->iADC;  // save new peak
    sig->PeakTimer = 0;       // restart peak timer
  } // new peak
  else // not new peak
  {
    if(sig->PeakTimer < tHold) // peak hold not elapsed
    {  
      sig->PeakTimer += sig->TimePerRead; // add elapsed time, leave peak alone
    }
    else // peak hold elapsed
    {
      sig->iADCPeak -= 10 * sig->TimePerRead;  // Linear decay of of peak
    }
  } // not new peak
} // ADC_Peak

// Entered at TimeIncement intervals, 
// Read only one ADC value on each entry
// Each ADC read takes about 100 usec
// select ADC based on 4 bit counter and mask
//  Id  every other, on odd counts
//  Pr  every 4th, on 2, 6, 10, 14
//  Pf  on 0, T on 4, Vs on 8, Vd on 12
// called from interrupt context
//-----------------------------------------
void UpdateAnalogInputs() 
{
  ADC_Read(&Pr);  // Reflected Power  
  ADC_Peak(&Pr);  
  ADC_Read(&Pf); // Forward Power    
  ADC_Peak(&Pf);
} // UpdateAnalogInputs()

//********************************************************************************
// Enter at TimeInterval = 2 ms
const int TimeBetweenDisplayUpdates = 50;  // msec
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

// Todo
// Calibrate power detectors
// Sequencer uses relay timing values for close, drop


//------------LcdPrintFloat---------------
// from: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1164927646
void LcdPrintFloat(float value, int places) 
{
  // this is used to cast digits
  int digit;
  float tens = 0.1;
  int tenscount = 0;
  int i;
  float tempfloat = value;

  // if value is negative, set tempfloat to the abs value
  // make sure we round properly. this could use pow from
  //<math.h>, but doesn't seem worth the import
  // if this rounding step isn't here, the value  54.321 prints as
  //54.3209

  // calculate rounding term d:   0.5/pow(10,places)
  float d = 0.5;
  if (value < 0)
	d *= -1.0;
  // divide by ten for each decimal place
  for (i = 0; i < places; i++)
	d/= 10.0;
  // this small addition, combined with truncation will round our
  // values properly
  tempfloat +=  d;

  // first get value tens to be the large power of ten less than value
  // tenscount isn't necessary but it would be useful if you wanted
  // to know after this how many chars the number will take

  if (value < 0)
	tempfloat *= -1.0;
  while ((tens * 10.0) <= tempfloat) 
  {
    tens *= 10.0;
    tenscount += 1;
  }

// write out the negative if needed
  if (value < 0)
	lcd.print('-');

  if (tenscount == 0)
	lcd.print(0, DEC);

  for (i=0; i< tenscount; i++) 
  {
    digit = (int) (tempfloat/tens);
    lcd.print(digit, DEC);
    tempfloat = tempfloat - ((float)digit * tens);
    tens /= 10.0;
  }

  // if no places after decimal, stop now and return
  if (places <= 0)
	return;

  // otherwise, write the point and continue on
  lcd.print('.');

  // now write out each decimal place by shifting digits one by one
  // into the ones place and writing the truncated value
  for (i = 0; i < places; i++) 
  {
	tempfloat *= 10.0;
	digit = (int) tempfloat;
	Serial.print(digit,DEC);
	// once written, subtract off that digit
	tempfloat = tempfloat - (float) digit;
  }
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
// blanks all the characters and returns cursor to upper left
void LcdBlankScreen()
{
  lcd.setCursor(0,0);
  lcd.print( "                ");
  lcd.setCursor(0,1);
  lcd.print( "                ");
  lcd.setCursor(0,0);
}
  
//**********************************************************************
// Display peak values
// This called from background, not ISR
// Convert to floating point, calibrate, display
// Manage peak hold and smoothing update rates
// converting ADC int to calibrated float takes 60 usec
// dBm to W conversion takes 380 usec
// called from main loop
//------------------------------------------------------
void DisplayValues() 
{
  int iTemp;
  char DisplayLine[17];  // 16 char, plus new line?
  
  LcdBlankScreen();
  
  // Pf  Power forward
  noInterrupts();
  iTemp = Pf.iADCPeak;
  interrupts();
  Pf.fVal  =  Pf.fCal * (float) iTemp-Vol_forward;
  SmoothDisplay(&Pf, Pf.fVal);
  lcd.print("Pf ");
  LcdPrintFloat(Pf.fVal, 0);
  lcd.print("W");
  
  // Pr  Power reverse
  noInterrupts();
  iTemp = Pr.iADCPeak;
  interrupts();
  Pr.fVal = Watts(Settings.CouplerGainFwdDB, Vol_forward, Pr.fCal * (float) iTemp-Vol_reverse); 
  //SmoothDisplay(&Pr, Pr.fVal);
  lcd.setCursor(0,1);
  lcd.print("Pr ");
  LcdPrintFloat(Pf.fVal, 0);
  lcd.print("W");
} //Display Values

//-----------------------------------------
// called from main loop
// used for debugging, etc.
void DisplayMessage() 
{
  lcd.setCursor(0,0);
  lcd.print("          ");
  lcd.setCursor(0,0);
}

//------------------------------------------
// called from main loop
void DisplayMeter() 
{
  //Pr and Pf .fval in Watts, convert to dBm
  int dBm;
  int pwm;
  dBm = 10*log(Pr.fVal)+30.0;
  pwm = (int) dBm * 4;
  if (pwm <   0) pwm =   0;
  if (pwm > 255) pwm = 255;
  analogWrite(MeterPinRev, pwm );
  
  dBm = 10*log(Pf.fVal)+30.0;
  pwm = (int) dBm * 4;
  if (pwm <   0) pwm =   0;
  if (pwm > 255) pwm = 255;
  analogWrite(MeterPinFwd, pwm );
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
    Serial.println(Pf.iADCPeak);
  }
}

//******************************************************************************
// Use cases
// Change power calibration
//  Press Select to initiate button mode, 
//    double click nothing, hold nothing
//  Press Left or Right to scroll to P.fwd calibration 
//    double click nothing, hold nothing
//  Display shows "Cal fwd 47.1 dB"
//  Press Up or Down to change calibration
//    single click, change by 0.1 dB
//    double click, change by 1.0 dB
//    hold, increment by 0.1 dB per second
//    long hold, increment by 1.0 dB per second
//  Wait
// Change Meter dynamics

//  Called from all the button places
void DisplayCurrentMode(int ModeIndex)
{
  char LcdString[17];
  LcdBlankScreen();
  switch (ModeIndex)
  {
    lcd.setCursor(0,1);
    case FwdMode:
      lcd.print(ModeNames[ModeIndex]);
      LcdPrintFloat(Settings.CouplerGainFwdDB, 5);
      lcd.print("dB");
      break;
    case RevMode:
      lcd.print(ModeNames[ModeIndex]);
      LcdPrintFloat(Settings.CouplerGainRevDB, 5);
      lcd.print("dB");
      break;
    case MeterMode:
      lcd.print(ModeNames[ModeIndex]);
      break;
    case BacklightMode:
      sprintf(LcdString, "Backlight %3d", (int) (100 * (float) Settings.BacklightLevel/ (float) BacklightLevelMax));
      lcd.print(LcdString);
      lcd.print("%");
      break;
    case FwdLimitMode:
      sprintf(LcdString, "Fwd Limit %4d W", Settings.LimitFwd);
      lcd.print(LcdString);
      break;
    case RevLimitMode:
      sprintf(LcdString, "Rev Limit %4d W", Settings.LimitRev);
      lcd.print(LcdString);
      break;
    default:
      { }
  }
}

//Group of Control function handlers
// Called with Up or Down Button Pressed
// Change between Power Display and Control Display handled externally
// Change between Control modes handles externally

//******************************************************************************
void FwdCalControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // Still pressed from entering Control mode
      case LeftButton:        // Still pressed
      case RightButton:       // Still pressed
        break;
      case UpButton:
        Settings.CouplerGainFwdDB += 0.1;
        if (Settings.CouplerGainFwdDB > CouplerGainFwdDBMax)
          { Settings.CouplerGainFwdDB = CouplerGainFwdDBMax; }
        DisplayCurrentMode(ModeIndex);    // Display info for new mode
        break;
      case DownButton:          
        Settings.CouplerGainFwdDB -= 0.1;
        if (Settings.CouplerGainFwdDB < CouplerGainFwdDBMin)
          { Settings.CouplerGainFwdDB = CouplerGainFwdDBMin; }
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);
  } // if (!NoButton)  
}

//******************************************************************************
void RevCalControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:            
        Settings.CouplerGainRevDB += 0.1;
        if (Settings.CouplerGainRevDB > CouplerGainRevDBMax)
          { Settings.CouplerGainRevDB = CouplerGainRevDBMax; }
        break;
      case DownButton:
        Settings.CouplerGainRevDB -= 0.1;
        if (Settings.CouplerGainRevDB < CouplerGainRevDBMin)
          { Settings.CouplerGainRevDB = CouplerGainRevDBMin; }     
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);
  } // if (!NoButton)  
}

//******************************************************************************
void MeterModeControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:     
        break;
      case DownButton:            
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);
  } // if (!NoButton)  
}

//******************************************************************************
// called in Control Mode for any button press
void BacklightControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:
        if (Settings.BacklightLevel < BacklightLevelMax) { Settings.BacklightLevel += 1; }
        analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness              
        break;
      case DownButton:
        if (Settings.BacklightLevel > 0) { Settings.BacklightLevel -= 1; }
        analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness              
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);    
  } // if (!NoButton)
} // BacklightControl()

//******************************************************************************
void FwdLimitControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // Still pressed from entering Control mode
      case LeftButton:        // Still pressed
      case RightButton:       // Still pressed
        break;
      case UpButton:
        Settings.LimitFwd += 1;
        if (Settings.LimitFwd > LimitFwdMax)
          { Settings.LimitFwd = LimitFwdMax; }
        DisplayCurrentMode(ModeIndex);    // Display info for new mode
        break;
      case DownButton:          
        Settings.LimitFwd -= 1;
        if (Settings.LimitFwd < LimitFwdMin)
          { Settings.LimitFwd = LimitFwdMin; }
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);
  } // if (!NoButton)  
}

//******************************************************************************
void RevLimitControl(int ButtonPressed, int ButtonPressTime)
{
  char LcdString[17];
  if (ButtonPressed != NoButton)  //skip display update most of the time
  {
    switch (ButtonPressed)
    {
      case SelectButton:      // pressed to exit Select mode
      case LeftButton:        // after changing to this mode from Left
      case RightButton:       // after changing to this mode from Right
        break;
      case UpButton:            
        Settings.LimitRev += 1;
        if (Settings.LimitRev > LimitRevMax)
          { Settings.LimitRev = LimitRevMax; }
        break;
      case DownButton:
        Settings.LimitRev -= 1;
        if (Settings.LimitRev < LimitRevMin)
          { Settings.LimitRev = LimitRevMin; }     
        break;
    } // switch(ButtonPressed)
    DisplayCurrentMode(ModeIndex);
  } // if (!NoButton)  
}


//******************************************************************************
// state machine
// Entered at timed intervals in background
//  Select, Left, Right, Up, Down

void DisplayMachine()
{
  GetKey();  // set value of ButtonPressed and ButtonPressTime
  switch (DisplayState)  // Control mode or Power Mode
  {
    case ControlMode:  
      switch (ButtonPressed)
      {   
        case NoButton:
          if (ButtonPressTime == ButtonPressTimeMax)
          {
            // Revert to Power display, erase control stuff from screen
            DisplayState = PowerMode;
            LcdBlankScreen();
          }
          break;  // NoButton

        case SelectButton:
          // return to power mode
          if (ButtonPressTime == 0)
          {
            // user selects end, write to EEPROM
            eeprom_write_block((const void*)&Settings, (void*)0, sizeof(Settings));
            DisplayState = PowerMode;
            LcdBlankScreen();
          }
          break;  // SelectButton
       
        case LeftButton:
          if (ButtonPressTime == 0)          // On leading edge of button press
          {
            // previous control mode, wrap around
            if (ModeIndex > ModeIndexMin) { ModeIndex -= 1; }  
            else if (ModeIndex <= ModeIndexMin) { ModeIndex = ModeIndexMax; }
          }
          DisplayCurrentMode(ModeIndex);    // Display info for new mode
          break;  // LeftButton

        case RightButton:
          if (ButtonPressTime == 0)          // On leading edge of button press
          {
            // next control mode, wrap around
            if (ModeIndex < ModeIndexMax) { ModeIndex += 1; }  // next control mode
            else if (ModeIndex >= ModeIndexMax) { ModeIndex = ModeIndexMin; }
          }
          DisplayCurrentMode(ModeIndex);    // Display info for new mode
          break;  // RightButton

        case UpButton:
        case DownButton:
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

      } // switch(ModeIndex
  
          break; 
        default:
          { }
      }  // switch(ButtonPressed)
      break;  // ControlMode

    case PowerMode:
      switch (ButtonPressed)
      {
        case NoButton:
          //
          DisplayValues();      // LCD numbers, P.fwd and P.rev
          DisplayMeter();       // PWM to meter or bar graph
          //DisplayMessage();     // 10 characters for debug or faults          
          break;
        case SelectButton:
          // return to power mode
          if (ButtonPressTime == 0)
          {
            DisplayState = ControlMode;
            DisplayCurrentMode(ModeIndex); 
          }
          break;
        case LeftButton:    // nothing
        case RightButton:   // nothing
        case UpButton:      // nothing
        case DownButton:    // nothing
        default:
          { }
      } // switch(ButtonPressed)
      break; // PowerMode
    default: 
      { }
    } // switch (DisplayState)
  } // StateMachine()


//*************************************************************************
// Called at TimeBetweenDisplayUpdates intervals (50 ms)
// 
void GetKey()
{
  adc_key_in = analogRead(0); // read the value from the sensor 
  key = get_key(adc_key_in);  // convert into key press
 
  if (key != oldkey)          // if keypress transition is detected
  {
    oldkey = key;
    ButtonPressTime = 0;
    if (key>=0)        // key pressed
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
    else // key = -1, not pressed
    {
      ButtonPressed = NoButton;
    } // if(key>= 0)
  }
  else // key == oldkey
  {
    ButtonPressTime += TimeBetweenDisplayUpdates;
    if (ButtonPressTime > ButtonPressTimeMax)
    {
      ButtonPressTime = ButtonPressTimeMax;
    } 
  } // if(key!=oldkey)
} // GetKey()

// Convert ADC value to key number
int get_key(unsigned int input)
{
    int k;
    for (k = 0; k < NUM_KEYS; k++)
    {
      if (input < adc_key_val[k]) { return k; }
    }
    if (k >= NUM_KEYS) { k = -1; } // No valid key pressed
    return k;
}

//*******************************************************************************
// Configure the pins, set outputs to standby
// Enable interrupts at the end
void setup() 
{
  // Setup pin modes
  pinMode( MeterPinFwd, OUTPUT);
  pinMode( MeterPinRev, OUTPUT);
  pinMode( FwdLimitPin, OUTPUT);
  pinMode( RevLimitPin, OUTPUT);
  pinMode(BacklightPin, OUTPUT);

  // read and fill in the calibration Settings values
  eeprom_read_block((void*)&Settings, (void*)0, sizeof(Settings));  

  analogWrite(BacklightPin, Settings.BacklightLevel);  // Set backlight brightness

  // set up the LCD's number of columns and rows: 
  lcd.clear();
  lcd.begin(16, 2);
  lcd.setCursor(0,0);

  LcdBlankScreen();
  lcd.print("  Power Meter");
  lcd.setCursor(0,1);
  lcd.print("   by WA1HCO");
  delay(5000);
  
  // The delay above give the ADC and LTC5507 time settle down (if necessary)

  // range check nonvolatile EEPROM values, set to mid if out of range
  CheckNonvolatile();

  // Button pin = A0 
  Pf.Pin     = A1;  // input pin for forward Power
  Pr.Pin     = A2;  // input pin for reverse Power
   
  // Calibrations, corresponds to max values
  // Calibration includes 
  //   AdcMaxVolts = 5.0
  //   Resistor voltage divider
  
  // Power Cal start with converting to volts
  Pf.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iADC to float Volts
  Pr.fCal = 1.0 / (float) AdcMaxCount * AdcMaxVolts; // iADC to float Volts

  // Limits in floating point
  //Pf.fValMax  = 1250.0;  // Watts
  //Pr.fValMax  =  100.0;  // Watts
   
  //Pf.iADCMax = Pf.fValMax / Pf.fCal; //TODO: not right
  //Pr.iADCMax = Pr.fValMax / Pr.fCal; //TODO: not right
  
  // convert gain of coupler from dB to linear
  CouplerGainFwdLinear = pow(10, Settings.CouplerGainFwdDB/10);
  CouplerGainRevLinear = pow(10, Settings.CouplerGainRevDB/10);
  
  // Read Forward and Reverse ADC values, assuming no RF to set Vol
  Vol_forward = Pf.fCal * (float) analogRead(Pf.iADC); // read the raw ADC value
  Vol_reverse = Pr.fCal * (float) analogRead(Pr.iADC); // read the raw ADC value  
  
  // Set initial state
  DisplayState = PowerMode;
  
  // Set initial button press value
  ButtonPressed = NoButton;
 
  // The serial port 
  Serial.begin(9600);
  
  // setup the timer and start it
  // timer used to read values and run state machine
  MsTimer2::set(TimeBetweenInterrupts, TimedService); // 2ms period
  // interrupts enabled after this point
  MsTimer2::start(); 
} // setup


//****************************************************************************
// Main Loop, just for display stuff
// Arduino falls into this after setup()
// This loop gets interrupted periodically
// All timed stuff occurs in ISR
// todo: cli(), sei() to manage interrupts
//---------------------------------------------
void loop() 
{
  // watch for signal from interrupt handler
  if (DisplayFlag == true) 
  {
    DisplayFlag = false;  // clear the flag
    DisplayMachine();     // state machine based on ButtonPressed
    serial();             // put info on serial port
  }
}
