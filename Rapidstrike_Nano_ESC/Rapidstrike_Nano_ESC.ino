#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <Servo.h>
#include "Bounce2.h"
#include <avr/pgmspace.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>
#include <EEPROM.h>

//Battery + "Bucked" down > 5ish volts in at +5
//Battery - > GND

//Trigger > D2

// Servo Signal (PULSE) > D9
// Servo Signal (PULSE) > D10

// Rotary Encoder on 11,12
// For your own sanity, you will need hardware debounce on the rotary encoder.


//----SETTINGS YOU CAN CHANGE
int BatteryS = 3; //2= 2s, 3= 3s, 4 = 4s 5 = 5s
#define MotorKV 3000 // motors KV. enter your kv here
int MaxMotorSpeed = 2000; // Reduce this if you want to electronically Limit your RPM / FPS .. 1000 being 0% 2000 being 100%
#define MotorSpinUpOverheadMS 100 // 150ms for motor to reach max speed from stall assuming no ramp up
byte MagSize = 18;

// Don't change the below setting 

// Arduino Pin Definitions
#define PIN_REVTRIGGER 7 // Rev trigger on digital pin 7
#define PIN_MAINMOTOR1 9  // ESC on digital pin 9
#define PIN_MAINMOTOR2 10  // ESC on digital pin 10
#define PIN_FIRETRIGGER 4  // Fire trigger on digital pin 4 - internal pullup resistor is shagged..
#define PIN_PUSHERRESET 6  // Pusher Reset on digital pin 6
#define PIN_LED 13
#define PIN_MOTORFET 8 // Maybe this can be used to engage the motors if needed
#define PIN_PUSHERMOTOR 5 // Pusher motor on PWM
#define PIN_OLED_SDA A4
#define PIN_OLED_SCL A5
#define PIN_MODE_SELECT_A 2 
#define PIN_MODE_SELECT_B 3
#define PIN_PUSHERFET 14
#define PIN_BATTERYDETECT A7
#define PIN_MAGRELEASE 15
#define PIN_JAMDOOR 16

// Servo Objects
Servo MainMotor1; 
Servo MainMotor2;
Servo PusherMotor;

// Acceleration Time (Not used - go max)
long AccelerateTime = 0; //ms. Start with 200ms for 4s /// Not used

// Deceleration Time
long DecelerateTime = 3000; //ms IF you have a bLHeli_32 OR S set this to 1000 otherwise 6000

// ESC Min Speed before shutoff
long MinBLHeliMotorSpeed = 1040;

// ESC Max Speed (n.b. Apparenly hitting 2000 causes latency.
long MaxBLHeliMotorSpeed = 1960;

// Servo Floor Speed
long MinMotorSpeed = 1000;

// Current Motor Speed
long CurrentMotorSpeed = MinMotorSpeed;

// Use this as a placeholder when motor direction changes before reaching end travel, and when lowering max speed 
long InterruptedMotorSpeed = 0;

// Speed Adjustment
long MaxMotorSpeedCeiling; //use this to remember the absolute max motor speed.
byte SetMaxSpeed = 100; // in percent.

// Track changes to Trigger State
unsigned long TimeLastTriggerChanged = 0;
unsigned long TimeLastTriggerPressed = 0;
unsigned long TimeLastTriggerReleased = 0;

// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireTriggerPressed = false; // Fire Trigger is Depressed
bool PusherResetPressed = false; // Pusher Reset is Depressed
bool MagReleasePressed = false; // Mag Release is Depressed
bool JamDoorPressed = false; // Mag Release is Depressed
unsigned long TimeLastPusherReset = 0;

// Main Motor Command
bool CommandRev = false;
bool PrevCommandRev = false; // So we can keep track of changes
bool AutoRev = false; // True when the computer is managing the rev process.

// Debounce Variables
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce RevTriggerBounce = Bounce();
Bounce FireTriggerBounce = Bounce();
Bounce PusherResetBounce = Bounce();
Bounce ModeSelectABounce = Bounce();
Bounce ModeSelectBBounce = Bounce();
Bounce MagReleaseBounce = Bounce();
Bounce JamDoorBounce = Bounce();

// Firing Control
#define MODE_SINGLESHOT 1
#define MODE_BURSTSHOT 2
#define MODE_FULLAUTO 3
byte CurrentMode = MODE_SINGLESHOT;  // We know what mode we are in
byte NumberOfDartsToShoot = 0; // How many darts to shoot in this batch. 255 means forever.
byte BurstLength = 3;
bool CommandFire = false;
bool PrevCommandFire = false;
bool AutoFire = false; // True when computer is firing
bool RunningFiringSequence = false;
bool PusherReset = false;
bool TriggerReset = true;
int DartsInMag = 0;
unsigned int TotalDartsFired = 0;
int PusherSpeed[] = {1300, 1600, 2000, 1300, 1200, 1400, 2000, 1200}; // 2S Slow, 2S Med, 2S High, 2S Single, 3S Slow, 3S Med, 3S High, 3S SIngle
int PusherStopMotorSpeed = 1000; 
byte PusherSpeedIndex = 1; // 0 = Slow, 1 = Med, 2 = High (Add base 3 for 3s)
bool PusherStopping = false;
#define MaxPusherResetTime 500
#define PusherRetractSpeed2s 1200
#define PusherRetractSpeed3s 1100

// Serial Comms
#define InputBufferMax 15
char SerialInputBuffer[InputBufferMax];
byte SavedMode = MODE_SINGLESHOT;
byte SavedBurstLength = 0;
bool HasSavedMode = false;

// Battery Monitoring
#define BATTERY_2S_MIN 6.0;
#define BATTERY_2S_MAX 8.4;
#define BATTERY_3S_MIN 9.0;
#define BATTERY_3S_MAX 12.6;
#define BATTERY_CALFACTOR 0.45 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage;
bool BatteryFlat = false;

// System Modes
#define MODE_CONFIG 1
#define MODE_MAGOUT 2
#define MODE_LOWBATTERY 3
#define MODE_NORMAL 4
int CurrentSystemMode = MODE_NORMAL;

// Display
SSD1306AsciiWire display;
#define OLED_ADDR 0x3C

// EEPROM Stuff
#define ADDR_ROF 0
#define ADDR_MAGSIZE 1
#define ADDR_MOTORSPEED 2
#define ADDR_BURST 3

void setup() {
  float MotorRPM;

  // Fooling about
  pinMode( PIN_MOTORFET, OUTPUT );
  digitalWrite( PIN_MOTORFET, 0 );
  pinMode( PIN_PUSHERFET, OUTPUT );
  digitalWrite( PIN_PUSHERFET, 0 );
  pinMode( PIN_LED, OUTPUT );

  // Set up comms
  Serial.begin(57600); 
  Serial.println( F("Booting.. ") );  

  // Boot LCD
  Serial.println( F("Initialising Display") );
  Wire.begin();
  Wire.setClock(400000L);
  display.begin(&Adafruit128x64, OLED_ADDR);
  Serial.println( F("Display Initialised") );

  digitalWrite(PIN_LED, HIGH );

  // Set up debouncing
  Serial.println( F("Configuring Debouncing") );
  pinMode(PIN_REVTRIGGER, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_REVTRIGGER );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_FIRETRIGGER, INPUT_PULLUP);
  FireTriggerBounce.attach( PIN_FIRETRIGGER );
  FireTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_PUSHERRESET, INPUT_PULLUP);
  PusherResetBounce.attach( PIN_PUSHERRESET );
  PusherResetBounce.interval( DebounceWindow );  

  pinMode(PIN_MODE_SELECT_A, INPUT_PULLUP);
  ModeSelectABounce.attach( PIN_MODE_SELECT_A );
  ModeSelectABounce.interval( DebounceWindow );
  
  pinMode(PIN_MODE_SELECT_B, INPUT_PULLUP);
  ModeSelectBBounce.attach( PIN_MODE_SELECT_B );
  ModeSelectBBounce.interval( DebounceWindow );  

  pinMode(PIN_MAGRELEASE, INPUT_PULLUP);
  MagReleaseBounce.attach( PIN_MAGRELEASE );
  MagReleaseBounce.interval( DebounceWindow );    

  pinMode(PIN_JAMDOOR, INPUT_PULLUP);
  JamDoorBounce.attach( PIN_JAMDOOR );
  JamDoorBounce.interval( DebounceWindow );  

  Serial.println( F("Debouncing Configured") );

  // Set up battery monitoring
  pinMode( PIN_BATTERYDETECT, INPUT );

  Serial.println( F("Initialising ESC") );
  // Set up motors
  MainMotor1.attach(PIN_MAINMOTOR1);
  MainMotor2.attach(PIN_MAINMOTOR2);
  PusherMotor.attach(PIN_PUSHERMOTOR);
  // Arm ESC's
  MainMotor1.writeMicroseconds(MinMotorSpeed);
  MainMotor2.writeMicroseconds(MinMotorSpeed);
  PusherMotor.writeMicroseconds(MinMotorSpeed);
  //delay(9000);   // Wait for ESC to initialise (9 seconds)
  Serial.println( F("ESC Initialised") );

  Serial.println( F("Loading EEPROM") );
  LoadEEPROM();
  Serial.println( F("EEPROM Loaded, Selecting Battery Type") );
  SetupSelectBattery();
  Serial.println( F("Battery Selected") );
  
  MaxMotorSpeedCeiling = MaxMotorSpeed;
  MotorRPM = BatteryS * MotorKV * 3.7;
  DecelerateTime = (float)(DecelerateTime) * (MotorRPM / 33300);
  if( BatteryS == 2 )
  {
    BatteryMinVoltage = BATTERY_2S_MIN;
    BatteryMaxVoltage = BATTERY_2S_MAX;
  }
  else
  {
    BatteryMinVoltage = BATTERY_3S_MIN;
    BatteryMaxVoltage = BATTERY_3S_MAX;
  }

  //digitalWrite( PIN_PUSHERFET, 1 );
  //digitalWrite( PIN_MOTORFET, 1 );
  digitalWrite( PIN_LED, 1 );
  delay( 1000 );
  digitalWrite( PIN_LED, 0 );
  //digitalWrite( PIN_MOTORFET, 0 );
  //digitalWrite( PIN_PUSHERFET, 0 );  

  Serial.println( F("Booted.") );

  // Finished booting, update display
  //display.clear();
}

void LoadEEPROM()
{
  bool CorruptData = false;

  SetMaxSpeed = EEPROM.read( ADDR_MOTORSPEED );
  PusherSpeedIndex = EEPROM.read( ADDR_ROF );
  BurstLength = EEPROM.read( ADDR_BURST );
  MagSize = EEPROM.read( ADDR_MAGSIZE );

  Serial.println( F("Read from EEPROM") );
  Serial.println( SetMaxSpeed );
  Serial.println( PusherSpeedIndex );
  Serial.println( BurstLength );
  Serial.println( MagSize );

  if( (SetMaxSpeed < 30) || (SetMaxSpeed > 100) ) CorruptData = true;
  if( (PusherSpeedIndex < 0) || (PusherSpeedIndex > 2) ) CorruptData = true;
  if( (BurstLength < 1) || (BurstLength > 50) ) CorruptData = true;
  if( (MagSize < 6) || (MagSize > 50) ) CorruptData = true;
  
  FireTriggerBounce.update();
  int TriggerStatus = FireTriggerBounce.read();
  if( (TriggerStatus == LOW) || CorruptData )
  {
    Serial.println( F("Something wrong with EEPROM or held trigger while booting") );
    Serial.println( CorruptData );
    Serial.println( TriggerStatus == LOW );
    Serial.println( (TriggerStatus == LOW) || CorruptData );    
    SetMaxSpeed = 75;
    PusherSpeedIndex = 1;
    BurstLength = 3;
    MagSize = 18;    

    EEPROM.write( ADDR_MOTORSPEED, SetMaxSpeed );
    EEPROM.write( ADDR_ROF, PusherSpeedIndex );
    EEPROM.write( ADDR_BURST, BurstLength );
    EEPROM.write( ADDR_MAGSIZE, MagSize );
  
  }

  Serial.println( F("Initialised") );
  Serial.println( SetMaxSpeed );
  Serial.println( PusherSpeedIndex );
  Serial.println( BurstLength );
  Serial.println( MagSize );

}

void SetupSelectBattery()
{
  unsigned long StartTime = millis();
  int BatSel = 0;
  int LastBatSel = 99;
  bool Processed = false;

  while( !Processed )
  {
    RevTriggerBounce.update(); // Update the pin bounce state
    FireTriggerBounce.update();

    if( RevTriggerBounce.fell() )
    {
      if( BatSel == 0 )
        BatSel = 1;
      else
        BatSel = 0;
    }
    if( FireTriggerBounce.fell() )
    {
      Processed = true;
    }
  
    if( BatSel != LastBatSel )
    {
      display.clear();
      display.setCursor(0, 0);
      display.setFont(ZevvPeep8x16);
      display.print( F("Confirm Battery\n") );
      if( BatSel == 0 )
      {
        display.print( F("> 2s\n") );
        display.print( F("  3s\n") );
        display.print( F("Rev Chg; Trig OK") );
      }
      else
      {
        display.print( F("  2s\n") );
        display.print( F("> 3s\n") );      
        display.print( F("Rev Chg; Trig OK") );
      }

    }
    LastBatSel = BatSel;
  }

  if( BatSel == 0 )
    BatteryS = 2;
  else
    BatteryS = 3;

  display.clear();
  display.setCursor(0, 0);
  display.print( F("Initialising") );
  
  while( millis() - StartTime < 9000 )
  {
    delay( 10 );
  }
}


/*
 * Read the sensor / button state for all buttons, including software debounce.
 */
void ProcessButtons()
{
  static bool LED = false;
  
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  FireTriggerBounce.update(); // Update the pin bounce state
  FireTriggerPressed = !(FireTriggerBounce.read());

  PusherResetBounce.update(); // Update the pin bounce state
  PusherResetPressed = !(PusherResetBounce.read() == LOW);
  if( PusherResetPressed )
    TimeLastPusherReset = millis();

  MagReleaseBounce.update(); // Update the pin bounce state
  MagReleasePressed = !(MagReleaseBounce.read());

  JamDoorBounce.update(); // Update the pin bounce state
  JamDoorPressed = !(JamDoorBounce.read());

  if( LED != FireTriggerPressed )
  {
    LED = FireTriggerPressed;
  }
  digitalWrite( PIN_LED, FireTriggerPressed );

  // Mode Select is handled by an On-Off-On switch
  ModeSelectABounce.update();
  ModeSelectBBounce.update();
  if( !CommandFire ) // Don't change while firing
  {
    // ModeA is ON, ModeB is OFF, Single Fire
    // ModeA is OFF, ModeB is OFF, Burst Fire
    // ModeA is Off, ModeB is ON, Full Auto
    bool ModeA = !(ModeSelectABounce.read());
    bool ModeB = !(ModeSelectBBounce.read());
    if( ModeA && !ModeB )
    {
      // Single Fire
      CurrentMode = MODE_SINGLESHOT;
    }
    else if( !ModeA && !ModeB )
    {
      // Burst Fire
      CurrentMode = MODE_BURSTSHOT;
    }
    else 
    {
      // Full Auto
      CurrentMode = MODE_FULLAUTO;
    }   
  }

  
}

/*
 * Slow the main motors to 0
 */
void RevDown()// RevDown 
{
  // Don't do anything if the motor is already stopped.
  if( CurrentMotorSpeed == MinMotorSpeed )
  {
    return;
  }

  if( DecelerateTime > 0 )
  {
    unsigned long CurrentTime = millis(); // Need a base time to calcualte from
    long SpeedRange = (MaxMotorSpeedCeiling - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
    long RampDownPerMS = SpeedRange / DecelerateTime;  // This is the number of units per S that we need to subtract
    unsigned long ActualDecelerationMS = CurrentTime - TimeLastTriggerReleased; // The number of MS in deceleration
    int NewMotorSpeed = MaxMotorSpeed - (ActualDecelerationMS * RampDownPerMS / 1000); // Calclate the new motor speed..

    // Now we need to take into account acceleration while braking
    if( NewMotorSpeed > CurrentMotorSpeed ) 
    {
      if( InterruptedMotorSpeed == 0 ) // Save the current motor speed for later comparison
      {
        InterruptedMotorSpeed = CurrentMotorSpeed;
        NewMotorSpeed = CurrentMotorSpeed;
      }
      else
      {
        NewMotorSpeed = InterruptedMotorSpeed - (MaxMotorSpeed - NewMotorSpeed);
      }      
    }

    if( NewMotorSpeed < MinMotorSpeed ) NewMotorSpeed = MinMotorSpeed; // Just in case we overshoot...

    if( (NewMotorSpeed - MinMotorSpeed) < (int)((float)MinMotorSpeed / 100 * 5) )
    {
      NewMotorSpeed = MinMotorSpeed; // We are within 5% of total stop, so just shut it down.
    }
    
    CurrentMotorSpeed = NewMotorSpeed;
  }
  else
  {
    // Immediately stop the motor
    CurrentMotorSpeed = MinMotorSpeed;
  }
}

/*
 * Run the main motors.
 */
void RevUp()
{

  // Don't do anything if the motor is already running.
  if( CurrentMotorSpeed == MaxMotorSpeed )
  {
    return;
  }

  if( AccelerateTime > 0 )
  {
    unsigned long CurrentTime = millis(); // Need a base time to calcualte from
    long SpeedRange = (MaxMotorSpeedCeiling - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
    long RampUpPerMS = SpeedRange / AccelerateTime;  // This is the number of units per S that we need to subtract
    unsigned long ActualAccelerationMS = CurrentTime - TimeLastTriggerPressed; // The number of MS in acceleration
    int NewMotorSpeed = MinMotorSpeed + (ActualAccelerationMS * RampUpPerMS / 1000); // Calclate the new motor speed..

    // Now we need to take into account acceleration while braking
    if( NewMotorSpeed < CurrentMotorSpeed ) 
    {
      if( InterruptedMotorSpeed == 0 ) // Save the current motor speed for later comparison
      {
        InterruptedMotorSpeed = CurrentMotorSpeed;
        NewMotorSpeed = CurrentMotorSpeed;
      }
      else
      {
        NewMotorSpeed = NewMotorSpeed - MinMotorSpeed + InterruptedMotorSpeed;
      }
    }

    if( NewMotorSpeed > MaxMotorSpeed ) NewMotorSpeed = MaxMotorSpeed; // Just in case we overshot...

    if( (NewMotorSpeed - MaxMotorSpeed) > (int)((float)MaxMotorSpeed / 100 * 95) )
    {
      NewMotorSpeed = MaxMotorSpeed; // We are within 5% of total full, so just wind it open.
    }
    
    CurrentMotorSpeed = NewMotorSpeed;
  }
  else
  {
    // Immediately hit the motor
    CurrentMotorSpeed = MaxMotorSpeed;
  }
}

/*
 * Account for changes to the rotary encoder and feed in a new max motor speed.
 */
void ProcessManualSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 10% and 100%
  
  MaxMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeedCeiling );  // Keep processing against the max theoretical motor speed.

  LastSetMaxSpeed = SetMaxSpeed;

  // Need to simulate pressing the trigger to get the smooth ramping
  if( CommandRev ) 
  {
    TimeLastTriggerPressed = millis();   
    InterruptedMotorSpeed = 0;
  }

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

}



/*
 * The main loop routine
 */
void loop() 
{

  // Check Battery Health
  ProcessBatteryMonitor();
  ProcessDisplay();
  ProcessSystemMode();

  // Process the button input
  ProcessButtons();
  ProcessMagRelease();

  // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }

  // Only process firing input commands when in MODFE_NORMAL.. Otherwise allow for current cycle to finish.
  if( CurrentSystemMode == MODE_NORMAL )
  {
    // Perform command logic
    ProcessRevCommand();
    ProcessTriggerCommand();    
    
  } 

  // Calculate new timing variables
  if( CommandRev != PrevCommandRev )
  {
    InterruptedMotorSpeed = 0; // Reset the interrupted motor speed
    TimeLastTriggerChanged = millis();
    PrevCommandRev = CommandRev;
    if( CommandRev )  // Was depressed, now pressed
    {
      TimeLastTriggerPressed = TimeLastTriggerChanged;
    }
    else // Was pressed, now depressed
    {
      TimeLastTriggerReleased = TimeLastTriggerChanged;
    }
  }

  // Process speed control override  
  ProcessManualSpeedControl();


  // Command main motors
  if( CommandRev ) // Commanding the motors to spin
  {
    RevUp();
  }
  else // Commanding the motors to decelerate
  {
    RevDown();
  }
  ProcessMainMotors();
  ProcessFiring();
  ProcessPusherReturn();
  

}

/*
 * Read Serial input - buffer command until we reach the termination character:
 * Return TRUE when there is a full command loaded, FALSE when not.
 * Protocol:
 * Start Character: #
 * Termination Character: $
 * Command Codes: 2 characters exactly
 * Commands:
 * Run Motor:  RM
 * Stop Motor:  SM
 * Set Max Speed: MS-XXX where XXX is the percentage of max speed
 * Query Device:  QD
 * Get Info: QS
 * Single Shot:  SS
 * Burst Shot: BS-XX where XX is the number of shots to fire
 * Pusher Speed: PS-X where X is 0 for low, 1 for med, 2 for high
 * Mag Size:  DM-XX where XX is the mag size (Min 6, Max 50)
 * 
 */
bool ProcessSerialInput()
{
  if( Serial.available() == 0 ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( Serial.available() > 0 )
  {
    char NextByte = Serial.read();

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= InputBufferMax ) CurrentBufferPosition = (InputBufferMax - 1);  // Capture Overflows.
    }
  }

  return false;
}

/*
 * A command was detected on the serial port. Find out what it is and execute it.
 */
void ProcessSerialCommand()
{
  char CommandHeader[3]; // Place the header into this buffer
  // Copy it using a lazy way
  CommandHeader[0] = SerialInputBuffer[0];
  CommandHeader[1] = SerialInputBuffer[1];
  CommandHeader[2] = 0;
  
  // Run Motor Command - RM
  if( (strcmp( CommandHeader, "RM" ) == 0) && (CurrentSystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = true;
  }

  // Stop Motor Command - SM
  if( (strcmp( CommandHeader, "SM" ) == 0) && (CurrentSystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = false;
  }

  // Set Max Speed - MS
  if( strcmp( CommandHeader, "MS" ) == 0 )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    SetMaxSpeed = constrain( atoi( IntValue ), 30, 100 );
    EEPROM.write( ADDR_MOTORSPEED, SetMaxSpeed );
  }    

  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    Serial.println( F("#RS-OK$") );
  }

  // Get Info Command - GI
  if( strcmp( CommandHeader, "GI" ) == 0 )
  {
    // GI-100-1-18-18-00000
    char SpeedBuffer[6];
    sprintf(SpeedBuffer, "%03d", SetMaxSpeed);
    Serial.print( F("#GI-") );
    Serial.print( SpeedBuffer[0] );
    Serial.print( SpeedBuffer[1] );
    Serial.print( SpeedBuffer[2] );
    sprintf(SpeedBuffer, "%d", PusherSpeedIndex);
    Serial.print( "-" );
    Serial.print( SpeedBuffer[0] );
    sprintf(SpeedBuffer, "%02d", MagSize);
    Serial.print( "-" );
    Serial.print( SpeedBuffer[0] );
    Serial.print( SpeedBuffer[1] );
    sprintf(SpeedBuffer, "%02d", DartsInMag);
    Serial.print( "-" );
    Serial.print( SpeedBuffer[0] );
    Serial.print( SpeedBuffer[1] );
    sprintf(SpeedBuffer, "%05d", TotalDartsFired);
    Serial.print( "-" );
    Serial.print( SpeedBuffer[0] );
    Serial.print( SpeedBuffer[1] );
    Serial.print( SpeedBuffer[2] );
    Serial.print( SpeedBuffer[3] );
    Serial.print( SpeedBuffer[4] );
    Serial.println( F("$") );
  }

  // Single Shot Command - SS
  if( (strcmp( CommandHeader, "SS" ) == 0) && (CurrentSystemMode == MODE_NORMAL) )
  {
    CommandFire = true;
    HasSavedMode = true;
    SavedMode = CurrentMode;
    SavedBurstLength = BurstLength;
    CurrentMode = MODE_SINGLESHOT;
    //TriggerReset = true;
    AutoFire = true;
  }

  // Burst Shot Command - BS
  if( (strcmp( CommandHeader, "BS" ) == 0) && (CurrentSystemMode == MODE_NORMAL) )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    CommandFire = true;
    HasSavedMode = true;
    SavedMode = CurrentMode;
    SavedBurstLength = BurstLength;
    CurrentMode = MODE_BURSTSHOT;
    BurstLength = constrain( atoi( IntValue ), 1, 99 );
    //TriggerReset = true;
    AutoFire = true;
    Serial.println( F("Autopilot - BS") );
  }   

  // Mag Size Command - DM
  if( strcmp( CommandHeader, "DM" ) == 0 )
  {
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    MagSize = constrain( atoi( IntValue ), 6, 50 );
    EEPROM.write( ADDR_MAGSIZE, MagSize );
  }   

  // Set ROF
  if( strcmp( CommandHeader, "PS" ) == 0 )
  {
    char IntValue[2] = { SerialInputBuffer[3], 0 };
    PusherSpeedIndex = constrain( atoi( IntValue ), 0, 3 );
    EEPROM.write( ADDR_ROF, PusherSpeedIndex );
  } 
}

/*
 * Process any logic in respect to the main motor control
 * Disengage main motor autopilot if user intervention was detected
 * Or engage auto pilot 
 */
void ProcessRevCommand()
{
  
  static bool PreviousRevTriggerPressed = false; // Keep track of the human input

  if( PreviousRevTriggerPressed != RevTriggerPressed )
  {
    // Human has taken control - disengage autopilot
    PreviousRevTriggerPressed = RevTriggerPressed;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    CommandRev = RevTriggerPressed; // We are in human control - just accept the trigger input
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

void ProcessTriggerCommand()
{
 static bool PreviousFireTriggerPressed = false; // Keep track of the human input 

  if( !FireTriggerPressed )
  {
    TriggerReset = true; // Reset the trigger
  }

  if( PreviousFireTriggerPressed != FireTriggerPressed )
  {
    // Human has taken control - disengage autopilot
    PreviousFireTriggerPressed = FireTriggerPressed;
    AutoFire = false;
    Serial.println( F("Fire state changed") );
    Serial.print( F("Trigger status = ") );
    Serial.println( FireTriggerPressed );
    if( FireTriggerPressed && (NumberOfDartsToShoot > 0) )
    {
      NumberOfDartsToShoot = 1; // Take control, fire the last dart, and let the motors shut down cleanly
      if( HasSavedMode ) // Restore the previous mode if overridden
      {
        BurstLength = SavedBurstLength;
        CurrentMode = SavedMode;
        HasSavedMode = false;
      }
    }
  }

  if( !AutoFire )
  {
    CommandFire = FireTriggerPressed; // We are in human control - just accept the trigger input
  }
  // Else the computer is controlling, and the current fire trigger state is ignored. Autopilot will adjust CommandFire
}

/*
 * Communicate with the main motors
 */
void ProcessMainMotors()
{
  static long PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    MainMotor1.writeMicroseconds( CurrentMotorSpeed );
    MainMotor2.writeMicroseconds( CurrentMotorSpeed );
    // Debugging output
    Serial.println(CurrentMotorSpeed);

    if( (CurrentMotorSpeed >= MaxMotorSpeed) || (CurrentMotorSpeed > PreviousMotorSpeed) )
    {
      digitalWrite( PIN_MOTORFET, 1 );
    }
    else
    {
      digitalWrite( PIN_MOTORFET, 0 );
    }

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Process Firing Commands
 */
bool FinishedFullAuto = false;
void ProcessFiring()
{
  // We want to do a couple of things here
  // 1 - Register the number of darts to fire
  // 1a - If full auto mode, make it 99 and reset back to 0 when the firing sequence has finished.
  // 2 - Determine if the motors are running. If not, spin the motors up, wait for the acceleration time + overhead and commence firing
  // 3 - Actuate the pusher motor
  // 4 - Every time the pusher return hits, decrement number of darts to fire by 1.
  // 5 - When you get to zero, stop the pusher motor on the next reset
  // 6 - If the motors were spun up programatically, stop them.


  //static bool FinishedFullAuto = false;
  static bool AutoPilot = false;
  static unsigned long MotorStartTime = 0;
  static bool RequestingFiringSequence = false; // We are requesting the start of a firing sequence.

  // Check the fire command.. If it's down, we are requesting the firing sequence... Need to do something else to have auto-pilot..
  if( CommandFire )
  {
    if( TriggerReset ) // Let's start
    {
      RequestingFiringSequence = true;
      // We are winding down the motors.
      // Either automatically, or manually.
      if( !CommandRev ) {
        RunningFiringSequence = false;   
        FinishedFullAuto = false;        
      }
    }
  }
  else // Fire Command is disabled.
  {
    if( (CurrentMode == MODE_FULLAUTO) && RunningFiringSequence ) // Trigger was let go in full auto mode
    {
      if( !FinishedFullAuto )
      {
        if( NumberOfDartsToShoot > 1 )
          NumberOfDartsToShoot = 1; // Fire the last dart, and let the motors shut down cleanly.. But don't increase dart count it already 0 or 1
        FinishedFullAuto = true;  
        //CommandFire = false;
        //Serial.println( "Full auto mode - trigger released" );      
      }
    }
    TriggerReset = true;
  }

  if( !RequestingFiringSequence ) // Actually, we don't want to fire.
    return;

  //TriggerReset = false;
  
  // Initialise the firing sequence
  if( !RunningFiringSequence ) 
  {
  
    // Determine the number of darts to fire.
    if( CurrentMode == MODE_SINGLESHOT )
    {
      NumberOfDartsToShoot = 1;
    }
    else if( CurrentMode == MODE_BURSTSHOT )
    {
      NumberOfDartsToShoot = BurstLength;
    }
    else if( CurrentMode == MODE_FULLAUTO )
    {
      NumberOfDartsToShoot = 99;
    }

    // Determine if the motors need to be run
    if( CurrentMotorSpeed < MaxMotorSpeed ) 
    {
      CommandRev = true;
      AutoPilot = true;  // Autopilot for shooting
      AutoRev = true; 
    }
    else
    {
      // Check delay
      if( MotorStartTime == 0 ) MotorStartTime = millis(); //Start watching for the delay

      if( (millis() - MotorStartTime >= MotorSpinUpOverheadMS) || !AutoPilot )  // Wait for the delay, otherwise if not in auto pilot, then start firing immediately.
      {
        // Begin firing! pew pew!
        RunningFiringSequence = true;
        TriggerReset = false;
        MotorStartTime = 0; // Reset back to zero      
        Serial.println( F("First shot now") ); 
      }
    }

    return; // Finished processing
  }

  // We have finished firing
  if( NumberOfDartsToShoot == 0 )
  {
    //TriggerReset = false; // Cannot commence firing until the trigger is reset
    FinishedFullAuto = false;  
    AutoFire = false;
    CommandFire = false;    
    if( AutoPilot ) // We are in auto pilot
    {
      CommandRev = false; // Command the motors to slow down   
      if( CurrentMotorSpeed <= MinMotorSpeed ) // We  have slowed to a stop
      {
        AutoPilot = false; // Disengage AutoPilot
      }
    }
    else // Reset back to normal and exit
    {      
      RunningFiringSequence = false;
      RequestingFiringSequence = false;
      if( HasSavedMode ) // Restore the previous mode if overridden
      {
        BurstLength = SavedBurstLength;
        CurrentMode = SavedMode;
        HasSavedMode = false;
      }
      Serial.println( F("Finished shooting") );
    }
    return;
  }
  else
  {
    // So, fire please.
    //static bool TestFire = true;
    //if( TestFire )
    StartPusher( (NumberOfDartsToShoot == 1) );   
    //TestFire = false; 
  }
  
  
  
}

/*
 * Process When the Pusher Returns Home
 */
void ProcessPusherReturn()
{
  static bool LastPusherResetPressed = true;

  //static bool CurrentPusherReset = true;
  //if( PusherResetBounce.fell() && !CurrentPusher

  if( PusherResetPressed != LastPusherResetPressed ) // We've detected a change
  {
    LastPusherResetPressed = PusherResetPressed; // Keep track of the pusher status
    if( !PusherResetPressed ) // The pusher is not pressed when it's in the home position
    {
      if( RunningFiringSequence ) // We are currently running a firing sequence
      {
        if( (CurrentMotorSpeed < MaxMotorSpeed) || !CommandRev ) // Check to make sure the motors are spinning at max before pushing another dart in.
        {
          Serial.println( F("Abort firing - motors stopped") );
          NumberOfDartsToShoot = 0;
          PusherStopping = true;
          StopPusher();
          if( DartsInMag > 0 )
          {
            DartsInMag --; // Count down any misfires though  
            TotalDartsFired ++;
          }
          return;          
        }
        Serial.println( F("Pusher is home while firing") );
        if( NumberOfDartsToShoot > 1 )
        {
          NumberOfDartsToShoot --; // Decrease the number of darts to fire.
          Serial.print( F("Darts remaining: ") );
          Serial.println( NumberOfDartsToShoot );
        }
        else
        {
          NumberOfDartsToShoot = 0;
          Serial.println( F("Halt pusher") );
          PusherStopping = true;
          StopPusher();          
        }
        if( DartsInMag > 0 ) // Decrement the known darts.
        {
          DartsInMag --;
          TotalDartsFired ++;
        }
        
      }
    }
  }
  
}

void StartPusher(bool LastShot)
{
  digitalWrite( PIN_PUSHERFET, 1 );

  PusherStopping = false;
  int SelectedIndex;
  int BaseIndex = 0;
  
  if( BatteryS == 3 )
    BaseIndex = 4;
  if( CurrentMode == MODE_SINGLESHOT )
  {
    SelectedIndex = 3;
  }
  else if( LastShot )
  {
    SelectedIndex = 0;
  }
  else
  {
    SelectedIndex = PusherSpeedIndex;
  }
  SelectedIndex += BaseIndex;

  PusherMotor.writeMicroseconds( PusherSpeed[SelectedIndex] );
}

void StopPusher()
{
  digitalWrite( PIN_PUSHERFET, 0 );  
  Serial.println( F("Pusher Stopped!!") );
  PusherMotor.writeMicroseconds( PusherStopMotorSpeed );

return;
  if( PusherStopping ) // We want to make sure the pusher resets back
  {
    PusherResetBounce.update();
    if( (PusherResetBounce.read() == LOW) ) //|| (millis() - TimeLastPusherReset > MaxPusherResetTime) ) // Currently not in reset position, or close enough
    {
      while( !PusherResetBounce.fell() )
      {
        PusherMotor.writeMicroseconds( 1100 );
        delay( 10 );
        PusherResetBounce.update();
      }
    }
    PusherMotor.writeMicroseconds( PusherStopMotorSpeed );
  }
}

void ProcessMagRelease()
{
  static bool LastMagReleasePressed = false;
  if( LastMagReleasePressed != MagReleasePressed )  // Detect a change in the status quo
  {
    if( MagReleasePressed )  // A mag has been inserted
    {
      DartsInMag = MagSize;
    }
    else // Mag has been dropped
    {
      DartsInMag = 0;
    }
  }
  LastMagReleasePressed = MagReleasePressed;
}

void ProcessBatteryMonitor()
{
  //return;
  #define NUM_SAMPLES 100
  static int CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERYDETECT );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47 + 22) / 22)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.0 ) // If the current voltage is 0, we are probably debugging
      {
        BatteryFlat = true;
      }
      else
      {
        BatteryFlat = false;
      }
    }
    else
    {
      BatteryFlat = false;
    }    
    CollectedSamples = 0;
    SampleAverage = 0;
  }
}

void ProcessSystemMode()
{
  if( !JamDoorPressed ) // Jam Door Open
  {
    CurrentSystemMode = MODE_CONFIG;
    return;
  }

  if( BatteryFlat ) // Battery low
  {
    CurrentSystemMode = MODE_LOWBATTERY;
    return;
  }

  if( !MagReleasePressed ) // Mag is out
  {
    CurrentSystemMode = MODE_MAGOUT;
    return;
  }

  CurrentSystemMode = MODE_NORMAL;
}

void ProcessDisplay()
{
  static int LastSystemMode = MODE_NORMAL;
  bool ClearScreen = false;
  
  if( LastSystemMode != CurrentSystemMode )
  {
    ClearScreen = true;
    LastSystemMode = CurrentSystemMode;
  }

  Display_ScreenHeader( ClearScreen );
  switch( CurrentSystemMode )
  {
    case MODE_MAGOUT:
      Display_MagOut( ClearScreen );
      break;
    case MODE_CONFIG:
      Display_Config( ClearScreen );
      break;
    case MODE_LOWBATTERY:
      Display_LowBatt( ClearScreen );
      break;
    default:
      Display_Normal( ClearScreen );
      break;
  }
}

void Display_LowBatt( bool ClearScreen )
{
  if( ClearScreen )
  {
    Serial.println( F("LOW BATTERY!!") );
    Serial.println( BatteryCurrentVoltage );

    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);
    display.print( F("################\n") );
    display.print( F("# LOW BATTERY! #\n") );
    display.print( F("################") ); 
  }
}

void Display_MagOut( bool ClearScreen )
{
  if( ClearScreen )
  {
    Serial.println( F("MAG OUT!!") ); 
    
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);
    display.print( F("################\n") );
    display.print( F("# MAG DROPPED! #\n") );
    display.print( F("################") );      
  }

  PusherResetBounce.update();
  if( !(PusherResetBounce.read() == LOW) ) // Currently not in reset position, or close enough
  {
    if( BatteryS == 2 )
      PusherMotor.writeMicroseconds( PusherRetractSpeed2s );
    else
      PusherMotor.writeMicroseconds( PusherRetractSpeed3s );
  }
  else
  {
    PusherMotor.writeMicroseconds( PusherStopMotorSpeed );
  }
}

void Display_Config( bool ClearScreen )
{
  static byte MenuItem = 0;
  static byte LastMenuItem = 99;
  if( ClearScreen || (MenuItem != LastMenuItem) )
  {
    Serial.println( F("CONFIG SCREEN!!") );

    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);

    char BurstBuffer[3];
    sprintf( BurstBuffer, "%2d", BurstLength );
    char MagBuffer[3];
    sprintf( MagBuffer, "%2d", MagSize );    
    char SpeedBuffer[3];
    sprintf( SpeedBuffer, "%3d", SetMaxSpeed );    
    
    switch( MenuItem )
    {
      case 0:
        display.print( F("> Pwr: ") );
        display.print( SpeedBuffer );
        display.print( F("%   \n") );
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      \n") );
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   ") ); 
        break;
      case 1:
        display.print( F("  Pwr: ") );
        display.print( SpeedBuffer );
        display.print( F("%   \n") );
        display.print( F("> ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      \n") );
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   ") ); 
        break;
      case 2:
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      \n") );
        display.print( F("> Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   \n") ); 
        display.print( F("  Mag Size: ") );
        display.print( MagBuffer );
        display.print( F("   ") ); 
        break;
      default:
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      \n") );
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   \n") ); 
        display.print( F("> Mag Size: ") );
        display.print( MagBuffer );
        display.print( F("   ") ); 
        break;          
    }

    LastMenuItem = MenuItem;
  }
  
  if( RevTriggerBounce.fell() )
  {
    MenuItem++;
    if( MenuItem > 3 )
      MenuItem = 0;
  }
   

  
  if( FireTriggerBounce.fell() )
  {
    switch( MenuItem )
    {
      case 0:
        SetMaxSpeed -= 5;
        if( SetMaxSpeed < 30 )
          SetMaxSpeed = 100;
        EEPROM.write( ADDR_MOTORSPEED, SetMaxSpeed );
        break;
      case 1:
        PusherSpeedIndex ++;
        if( PusherSpeedIndex > 2 )
          PusherSpeedIndex = 0;
        EEPROM.write( ADDR_ROF, PusherSpeedIndex );  
        break;
      case 2:
        BurstLength ++;
        if( BurstLength > MagSize )
          BurstLength = 1;
        EEPROM.write( ADDR_BURST, BurstLength );
        break;
      default:
        MagSize ++;
        if( MagSize > 35 )
          MagSize = 6;
        EEPROM.write( ADDR_MAGSIZE, MagSize );
        break;
    }
    LastMenuItem = 99;
  }
    
}

void Display_Normal( bool ClearScreen )
{
  char Buffer[4];
  static int LastDartsInMag = 99;
  static byte LastCurrentMode = 99;
  static byte LastPusherSpeedIndex = 99;
  static int LastSetMaxSpeed = 99;
  static byte LastBurstLength = 99;
  if( ClearScreen )
  {
    Serial.println( F("NORMAL MODE!!") );
  }
  if( ClearScreen || (CurrentMode != LastCurrentMode) || (BurstLength != LastBurstLength) )
  {  
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);
    if( CurrentMode == MODE_SINGLESHOT )
    {
      display.print( F("Single    ") );
    }
    else if( CurrentMode == MODE_FULLAUTO )
    {
      display.print( F("Full Auto ") );
    }
    else
    {
      sprintf( Buffer, "%2d", BurstLength );
      display.print( F("Burst: ") );
      display.print( Buffer );
    }
  }
  if( ClearScreen || (PusherSpeedIndex != LastPusherSpeedIndex) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 4);
    display.print( F("ROF: ") );
    if( PusherSpeedIndex == 0 )
      display.print( F("L") );
    else if( PusherSpeedIndex == 1 )
      display.print( F("M") );
    else
      display.print( F("H") );
  }
  if( ClearScreen || (SetMaxSpeed != LastSetMaxSpeed) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 6);
    sprintf( Buffer, "%3d", SetMaxSpeed );
    display.print( F("Pwr: ") );
    display.print( Buffer );
    display.print( "%" );
  }
  if( ClearScreen || (DartsInMag != LastDartsInMag) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(90, 3);
    display.set2X();
    sprintf( Buffer, "%2d", DartsInMag );
    display.print( Buffer );
    display.set1X();  
  }
  LastDartsInMag = DartsInMag;
  LastBurstLength = BurstLength;
  LastSetMaxSpeed = SetMaxSpeed;
  LastCurrentMode = CurrentMode;
}

void Display_ScreenHeader( bool ClearScreen )
{
  float LastBatteryVoltage = 99.0;
  unsigned int LastTotalDartsFired = 9999;
  char Buffer[6];
  
  if( ClearScreen )
  {
    display.clear();
  }
  // Do not clear screen, voltage has changed
  if( ClearScreen || ( (int)(LastBatteryVoltage*10) != (int)(BatteryCurrentVoltage*10) ) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 0);
    sprintf( Buffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    Buffer[4] = 0;
    Buffer[3] = Buffer[2];
    Buffer[2] = '.';
    //sprintf( Buffer, "%2.1f", BatteryCurrentVoltage );
    display.print( Buffer );
    display.print( F("v ") );
    display.print( BatteryS );
    display.print( F("s") );
    //Serial.println( BatteryCurrentVoltage );
    //Serial.println( Buffer );
  }
  if( ClearScreen || (LastTotalDartsFired != TotalDartsFired ) )
  {
    display.setCursor( 70, 1 );
    sprintf(Buffer, "%5d", TotalDartsFired);
    display.setFont(font5x7);
    //display.setRow( 1 );
    display.print( "-> " );
    display.print( Buffer );      
  }

  LastBatteryVoltage = BatteryCurrentVoltage;
  LastTotalDartsFired = TotalDartsFired;
}
