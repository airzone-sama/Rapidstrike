#include <Bounce2.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include <Wire.h>


// Pin Definitions
#define PIN_SELECT_FIRE_A 7
#define PIN_SELECT_FIRE_B 8
#define PIN_TRIGGER_FULL 16
#define PIN_TRIGGER_REV 14
#define PIN_PUSHER_RESET 15
#define PIN_MOTOR_A 9
#define PIN_MOTOR_B 10
#define PIN_PUSHER_RUN 5
#define PIN_PUSHER_STOP 18
#define PIN_BATTERY_MONITOR A3
#define PIN_MAGRELEASE 19
#define PIN_JAMDOOR 20
#define PIN_OLED_SDA 2
#define PIN_OLED_SCL 3


// Configuration Options
byte BurstSize = 2;
byte MotorSpeedFull = 75; // For full-pull
byte MotorSpeedHalf = 30; // For half-pull
byte MagSize = 18;


// Modes
#define MODE_NORMAL 0
#define MODE_PUSHER_JAM 1 
#define MODE_MAGOUT 2
#define MODE_LOWBATTERY 3
#define MODE_CONFIG 4
byte SystemMode = MODE_NORMAL;


// Physical Switch Status
bool RevTriggerPressed = false; // Rev Trigger is Depressed
bool FireFullTriggerPressed = false; // Fire Trigger is Depressed
bool PusherResetPressed = false; // Fire Trigger is Depressed
bool MagReleasePressed = false; // Mag Release is Depressed
bool JamDoorPressed = false; // Mag Release is Depressed


// Firing Controls
#define FIRE_MODE_SINGLE 0
#define FIRE_MODE_BURST 1
#define FIRE_MODE_AUTO 2
#define FIRE_MODE_AUTO_LASTSHOT 3
#define FIRE_MODE_IDLE 4
byte CurrentFireMode = FIRE_MODE_SINGLE; // This is the user request based on the button state
byte ProcessingFireMode = FIRE_MODE_IDLE; // This is what will actually be fired.
bool ExecuteFiring = false; // Set to true when the Solenoid is supposed to move
int TimeBetweenShots = 0; // Calculated to lower ROF
long ShotsToFire = 0; // Number of shots in the queue
unsigned long LastShot = 0; // When the last shot took place.
bool RequestShot = false; // Set to true to request the firing sequence to commence
bool RequestAutoStop = false; // Set to true to stop Full Auto
int DartsInMag = 0;
bool FiringLastShot = false;
unsigned int TotalDartsFired = 0;
unsigned int LifetimeTotalDartsFired = 0;


// Motor Controls
#define MOTOR_SPINUP_LAG 300 // How long we give the motors before we know that have spun up.
#define MOTOR_SPINDOWN_2S 3000
#define MOTOR_SPINDOWN_3S 4000
#define MOTOR_SPINDOWN_4S 6000
#define MOTOR_SPINUP_2S 0
#define MOTOR_SPINUP_3S 0
#define MOTOR_SPINUP_4S 0
#define MOTOR_MAX_SPEED 2000
int MaxMotorSpeed = MOTOR_MAX_SPEED;
int DecelerateTime = 0;
int AccelerateTime = 0;
long MotorRampUpPerMS = 0;
long MotorRampDownPerMS = 0;
int MinMotorSpeed = 1000;
int CurrentMotorSpeed = MinMotorSpeed;
int TargetMotorSpeed = MinMotorSpeed;
byte SetMaxSpeed = 100; // in percent.
unsigned long TimeLastMotorSpeedChanged = 0;
#define COMMAND_REV_NONE 0
#define COMMAND_REV_HALF 1
#define COMMAND_REV_FULL 2
byte CommandRev = COMMAND_REV_NONE;
byte PrevCommandRev = COMMAND_REV_NONE;
bool AutoRev = false; // True when the computer is managing the rev process.


// Inputs
#define DebounceWindow 5 // Debounce Window = 5ms
Bounce FireFullTriggerBounce = Bounce();
Bounce RevTriggerBounce = Bounce();
Bounce PusherResetBounce = Bounce();
Bounce ModeSelectABounce = Bounce();
Bounce ModeSelectBBounce = Bounce();
Bounce MagReleaseBounce = Bounce();
Bounce JamDoorBounce = Bounce();


// Serial Comms
#define SERIAL_INPUT_BUFFER_MAX 30
char SerialInputBuffer[SERIAL_INPUT_BUFFER_MAX];
byte SavedMode = FIRE_MODE_SINGLE;
byte SavedBurstSize = 0;
bool HasSavedMode = false;
bool AutoFire = false;
int AutoFireMotorSpeed = 0;


// Battery Monitoring
#define BATTERY_2S_MIN 6.0
#define BATTERY_2S_MAX 8.4
#define BATTERY_3S_MIN 9.0
#define BATTERY_3S_MAX 12.6
#define BATTERY_4S_MIN 12.0
#define BATTERY_4S_MAX 16.8
#define BATTERY_CALFACTOR 0.0 // Adjustment for calibration.
float BatteryMaxVoltage;
float BatteryMinVoltage;
float BatteryCurrentVoltage;
bool BatteryFlat = false;
int BatteryPercent = 100;
byte BatteryS = 3;


// Pusher controls
bool PusherStopping = false;
bool PusherRetracting = false;
bool PusherPWMFETOn = false; // Keep track of the PWM FET
bool PusherBrakeFETOn = false; // Keep track of the brake fet
bool PusherRequest = false; // False = stop pusher, true = run pusher
#define PusherStop 0     // Pusher brake is on, PWM is off
#define PusherTransition 1  // Pusher brake is off, PWM is off, waiting for fet caps to discharge
#define PusherRun 2         // Pusher brake is off, PWM is on
byte CurrentPusherStatus = PusherTransition; // Start in transition mode for boot.
unsigned long PusherTransitionStart = 0; // Time transition started
#define PusherOnTransitionTime 10 // 100ms transition time 
#define PusherOffTransitionTime 2 // 100ms transition time 
unsigned long TimeLastPusherResetOrActivated = 0; // We are keeping track when the pusher was last reset for anti-jam purposes.
#define PusherMaxCycleTime 2000   // Max number of time between cycles before we recognise a jam
bool JamDetected = false;
bool RunningFiringSequence = false;
int PusherSpeedPWM[] = {40, 65, 100, 40, 25, 15, 
                        33, 55, 100, 33, 25, 15,
                        30, 50, 80, 30, 25,  15}; // 2s: Slow, Med, High, Single, last shot, retract; 3S; 4S
#define ROF_SLOW 0
#define ROF_MEDIUM 1
#define ROF_HIGH 2
#define ROF_SINGLE 3
#define ROF_LASTSHOT 4
#define ROF_RETRACT 5
byte PusherSpeedIndex = ROF_MEDIUM; // 0 = Slow, 1 = Med, 2 = High (Add base 3 for 3s)
byte PusherROF = 0; // ROF Percentage
byte GetPusherSpeed( byte PSI ) {return PusherSpeedPWM[ PSI + (BatteryS - 2) ];}  // Mini function to determine pusher speed based on battery sizr


// Display
SSD1306AsciiWire display;
#define OLED_ADDR 0x3C


// EEPROM Stuff
#define ADDR_ROF 0
#define ADDR_MAGSIZE 1
#define ADDR_HIGHMOTORSPEED 2
#define ADDR_LOWMOTORSPEED 6
#define ADDR_BURST 3
#define ADDR_TDF 4


void setup() {

  // Set up comms
  Serial.begin(57600); 
  Serial1.begin(57600);
  Serial.println( F("Booting.. ") );  

  // Boot LCD
  Serial.println( F("Initialising Display") );
  Wire.begin();
  Wire.setClock(400000L);
  display.begin(&Adafruit128x64, OLED_ADDR);
  Serial.println( F("Display Initialised") );  

  // Set up debouncing
  Serial.println( F("Configuring Debouncing") );  
  
  pinMode(PIN_TRIGGER_REV, INPUT_PULLUP);
  RevTriggerBounce.attach( PIN_TRIGGER_REV );
  RevTriggerBounce.interval( DebounceWindow );
  
  pinMode(PIN_TRIGGER_FULL, INPUT_PULLUP);
  FireFullTriggerBounce.attach( PIN_TRIGGER_FULL );
  FireFullTriggerBounce.interval( DebounceWindow );

  pinMode(PIN_PUSHER_RESET, INPUT_PULLUP);
  PusherResetBounce.attach( PIN_PUSHER_RESET );
  PusherResetBounce.interval( DebounceWindow );

  pinMode(PIN_SELECT_FIRE_A, INPUT_PULLUP);
  ModeSelectABounce.attach( PIN_SELECT_FIRE_A );
  ModeSelectABounce.interval( DebounceWindow );
  
  pinMode(PIN_SELECT_FIRE_B, INPUT_PULLUP);
  ModeSelectBBounce.attach( PIN_SELECT_FIRE_B );
  ModeSelectBBounce.interval( DebounceWindow ); 

  pinMode(PIN_MAGRELEASE, INPUT_PULLUP);
  MagReleaseBounce.attach( PIN_MAGRELEASE );
  MagReleaseBounce.interval( DebounceWindow );    

  pinMode(PIN_JAMDOOR, INPUT_PULLUP);
  JamDoorBounce.attach( PIN_JAMDOOR );
  JamDoorBounce.interval( DebounceWindow );  

  Serial.println( F("Debouncing Configured") );

  // Setup Motor Outputs
  Serial.println( F("Configuring PWM Ports") );
  pinMode( PIN_MOTOR_A, OUTPUT );
  pinMode( PIN_MOTOR_B, OUTPUT );
  digitalWrite( PIN_MOTOR_A, LOW );
  digitalWrite( PIN_MOTOR_B, LOW );
  TCCR1A = 0;
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = 0;
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
  ICR1 = 40000;
  UpdatePWM( 1000 );


  // Setup Pusher Outputs
  Serial.println( F("Configuring Pusher FET") );
  pinMode( PIN_PUSHER_RUN, OUTPUT );
  digitalWrite( PIN_PUSHER_RUN, LOW );  
  pinMode( PIN_PUSHER_STOP, OUTPUT );
  digitalWrite( PIN_PUSHER_STOP, LOW );  

  LoadEEPROM();
  Serial.println( F("Selecting Battery Type") );
  SetupSelectBattery();

  if( BatteryS == 2 )
  {
    Serial.println( F("Configuring for 2S Battery") );
    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;    

    BatteryMaxVoltage = BATTERY_2S_MAX;
    BatteryMinVoltage = BATTERY_2S_MIN;
  }
  else if( BatteryS == 3 )
  {
    Serial.println( F("Configuring for 3S Battery") );
    DecelerateTime = MOTOR_SPINDOWN_3S;
    AccelerateTime = MOTOR_SPINUP_3S;

    BatteryMaxVoltage = BATTERY_3S_MAX;
    BatteryMinVoltage = BATTERY_3S_MIN;
  }
  else
  {
    Serial.println( F("Configuring for 4S Battery") );
    DecelerateTime = MOTOR_SPINDOWN_4S;
    AccelerateTime = MOTOR_SPINUP_4S;

    BatteryMaxVoltage = BATTERY_4S_MAX;
    BatteryMinVoltage = BATTERY_4S_MIN;
  }

  SystemMode = MODE_NORMAL;
  CalculateRampRates(); 

  // Now wait until the trigger is high
  Serial.println( F("Waiting for trigger safety") );
  FireFullTriggerBounce.update();
  while( FireFullTriggerBounce.read() == LOW )
  {
    delay(10);
    FireFullTriggerBounce.update();
  }
  delay(10);

  Serial.println( F("Booted") );   
}


void LoadEEPROM()
{
  bool CorruptData = false;

  MotorSpeedFull = EEPROM.read( ADDR_HIGHMOTORSPEED );
  MotorSpeedHalf = EEPROM.read( ADDR_LOWMOTORSPEED );
  PusherSpeedIndex = EEPROM.read( ADDR_ROF );
  BurstSize = EEPROM.read( ADDR_BURST );
  MagSize = EEPROM.read( ADDR_MAGSIZE );
  EEPROM.get( ADDR_TDF, LifetimeTotalDartsFired );

  Serial.println( F("Read from EEPROM") );
  Serial.println( MotorSpeedFull );
  Serial.println( MotorSpeedHalf );
  Serial.println( PusherSpeedIndex );
  Serial.println( BurstSize );
  Serial.println( MagSize );
  Serial.println( LifetimeTotalDartsFired );

  if( (MotorSpeedFull < 30) || (MotorSpeedFull > 100) ) CorruptData = true;
  if( (MotorSpeedHalf < 30) || (MotorSpeedHalf > 100) ) CorruptData = true;
  if( (PusherSpeedIndex < 0) || (PusherSpeedIndex > 2) ) CorruptData = true;
  if( (BurstSize < 1) || (BurstSize > 50) ) CorruptData = true;
  if( (MagSize < 6) || (MagSize > 50) ) CorruptData = true;
  
  FireFullTriggerBounce.update();
  int TriggerStatus = FireFullTriggerBounce.read();
  if( (TriggerStatus == LOW) || CorruptData )
  {
    Serial.println( F("Something wrong with EEPROM or held trigger while booting") );
    Serial.println( CorruptData );
    Serial.println( TriggerStatus == LOW );
    Serial.println( (TriggerStatus == LOW) || CorruptData );    
    MotorSpeedFull = 75;
    MotorSpeedHalf = 45;
    PusherSpeedIndex = 1;
    BurstSize = 2;
    MagSize = 18;    
    LifetimeTotalDartsFired = 0;

    EEPROM.write( ADDR_HIGHMOTORSPEED, MotorSpeedFull );
    EEPROM.write( ADDR_LOWMOTORSPEED, MotorSpeedHalf );
    EEPROM.write( ADDR_ROF, PusherSpeedIndex );
    EEPROM.write( ADDR_BURST, BurstSize );
    EEPROM.write( ADDR_MAGSIZE, MagSize );
    EEPROM.put( ADDR_TDF, LifetimeTotalDartsFired );
  }

  Serial.println( F("Initialised") );
  Serial.println( MotorSpeedFull );
  Serial.println( MotorSpeedHalf );
  Serial.println( PusherSpeedIndex );
  Serial.println( BurstSize );
  Serial.println( MagSize );
  Serial.println( LifetimeTotalDartsFired );

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
    FireFullTriggerBounce.update();

    if( RevTriggerBounce.fell() )
    {
      if( BatSel == 0 )
        BatSel = 1;
      else if( BatSel == 1 )
        BatSel = 2;
      else
        BatSel = 0;
    }
    if( FireFullTriggerBounce.fell() )
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
        display.print( F("  4s") );
      }
      else if( BatSel == 1 )
      {
        display.print( F("  2s\n") );
        display.print( F("> 3s\n") );      
        display.print( F("  4s") );
      }
      else
      {
        display.print( F("  2s\n") );
        display.print( F("  3s\n") );
        display.print( F("> 4s") );        
      }

    }
    LastBatSel = BatSel;
  }

  if( BatSel == 0 )
    BatteryS = 2;
  else if( BatSel == 1 )
    BatteryS = 3;
  else
    BatteryS = 4;

  display.clear();
  display.setCursor(0, 0);
  display.print( F("Initialising") );
  
  while( millis() - StartTime < 9000 )
  {
    delay( 10 );
  }

  display.clear();
  display.setCursor(0, 0);
}

/*
 * This is a boot time init sub to calcualte the Acceleration and 
 * deceleration ramp rates of the motors.
 */
void CalculateRampRates()
{
  long SpeedRange = (long)(MaxMotorSpeed - MinMotorSpeed) * 1000; // Multiply by 1000 to give some resolution due to integer calculations
  if( AccelerateTime == 0 )
  {
    MotorRampUpPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampUpPerMS = SpeedRange / AccelerateTime;  // Use when Accelerating
  }

  if( DecelerateTime == 0 )
  {
    MotorRampDownPerMS = SpeedRange;  // For instant acceleration
  }
  else
  {
    MotorRampDownPerMS = SpeedRange / DecelerateTime;  // Use when Decelerating
  }  


  Serial.print( F("Ramp Up per MS = ") );
  Serial.println( MotorRampUpPerMS );

  Serial.print( F("Ramp Down per MS = ") );
  Serial.println( MotorRampDownPerMS );
}

// Updates the PWM Timers
void UpdatePWM( int NewSpeed )
{
  NewSpeed = (NewSpeed * 2) + 2; // Adjust for the prescalar
  OCR1A = NewSpeed;
  OCR1B = NewSpeed;
}

void loop() {
  // put your main code here, to run repeatedly:

  ProcessButtons(); // Get User and Sensor input
  ProcessBatteryMonitor(); // Check battery voltage
  ProcessSystemMode(); // Find out what the system should be doing
  ProcessMagRelease();
  ProcessDisplay();

    // Process Serial input
  if( ProcessSerialInput() )
  {
    ProcessSerialCommand();
  }
  
  ProcessRevCommand(); // Handle motor intentions
  
  // Detected a change to the command. Reset the last speed change timer.
  if( PrevCommandRev != CommandRev )
  {
    TimeLastMotorSpeedChanged = millis();
    PrevCommandRev = CommandRev;
  }
    
  // Process speed control  
  ProcessSpeedControl();
  // Calcualte the new motor speed
  ProcessMotorSpeed();
  // Send the speed to the ESC
  ProcessMainMotors();  


  ProcessFiring();
  ProcessPusherReturn();
  ProcessPusher();  

}

/*
 * Process input from Buttons and Sensors.
 */
void ProcessButtons()
{
  RevTriggerBounce.update(); // Update the pin bounce state
  RevTriggerPressed = !(RevTriggerBounce.read());

  PusherResetBounce.update(); // Update the pin bounce state
  PusherResetPressed = !(PusherResetBounce.read());

  FireFullTriggerBounce.update(); // Update the pin bounce state
  FireFullTriggerPressed = !(FireFullTriggerBounce.read()); 
  RequestShot = FireFullTriggerBounce.fell(); // Programatically keep track of the request for a shot
  RequestAutoStop = FireFullTriggerBounce.rose();

  MagReleaseBounce.update(); // Update the pin bounce state
  MagReleasePressed = !(MagReleaseBounce.read());

  JamDoorBounce.update(); // Update the pin bounce state
  JamDoorPressed = !(JamDoorBounce.read());

  // Determine the current firing mode
  ModeSelectABounce.update();
  ModeSelectBBounce.update();
  if( !AutoFire )
  {
    if( ModeSelectABounce.read() == LOW && ModeSelectBBounce.read() == HIGH && CurrentFireMode != FIRE_MODE_AUTO_LASTSHOT )
      CurrentFireMode = FIRE_MODE_AUTO;
    else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == HIGH )
      CurrentFireMode = FIRE_MODE_BURST;
    else if( ModeSelectABounce.read() == HIGH && ModeSelectBBounce.read() == LOW )
      CurrentFireMode = FIRE_MODE_SINGLE;
  }
}


// We are toggline between different system states here..
// Also handle the blasted configuration controls here... Because there's no special configuration screen
void ProcessSystemMode()
{
  static byte LastSystemMode = MODE_NORMAL;

  if( !JamDoorPressed ) // Jam Door Open
  {
    SystemMode = MODE_CONFIG;
  }
  
  else if( BatteryFlat ) // Battery low
  {
    SystemMode = MODE_LOWBATTERY;
  }

  else if( !MagReleasePressed ) // Mag is out
  {
    SystemMode = MODE_MAGOUT;
  }

  else if( JamDetected ) // Jam is detected
  {
    SystemMode = MODE_PUSHER_JAM;
    
    if( LastSystemMode != MODE_PUSHER_JAM )
    {
      SystemMode = MODE_PUSHER_JAM;

      ShotsToFire = 0;
      Serial.println( F("Halt pusher") );
      PusherStopping = true;
      StopPusher(); 

      ExecuteFiring = false;
      if( AutoFire )
      {
        AutoFire = false;
        if( HasSavedMode )
        {
          BurstSize = SavedBurstSize;
          CurrentFireMode = SavedMode;
          HasSavedMode = false;
        }
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE;
      }
      else
      {
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE; 
        RequestShot = false;      
        if( HasSavedMode )
        {
          BurstSize = SavedBurstSize;
          CurrentFireMode = SavedMode;
          HasSavedMode = false;
        }
      }
      
      Serial.println( F("Pusher Jam Detected") );  
      
    }
    
  }
  
  else 
  {
    SystemMode = MODE_NORMAL;
  }

  if( LastSystemMode != SystemMode )
  {
    Serial.print( F("New System Mode = ") );
    Serial.println( SystemMode );
    LastSystemMode = SystemMode;
  }
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
  if( (Serial.available() == 0) && (Serial1.available() == 0) ) return false; // Ignore when there is no serial input
  
  static byte CurrentBufferPosition = 0;

  while( (Serial.available() > 0) || (Serial1.available() > 0) )
  {
    char NextByte = 0;
    if( Serial.available() > 0 )
    { 
      NextByte = Serial.read();
      //Serial1.write( NextByte );
    }
    else if( Serial1.available() > 0 )
    {
      NextByte = Serial1.read();
      //Serial.write( "x" );
    }

    switch( NextByte )
    {
      case '#': // Starting new command
        CurrentBufferPosition = 0;
        break;
      case '$': // Ending command
        return true; // Jump out.. There's more data in the buffer, but we can read that next time around.
        break;
      case '?': // Presume help - Simulate DS
        SerialInputBuffer[0] = 'D';
        SerialInputBuffer[1] = 'S';
        return true;
        break;
      default: // Just some stuff coming through
        SerialInputBuffer[ CurrentBufferPosition ] = NextByte; // Insert into the buffer
        CurrentBufferPosition ++; // Move the place to the right
        if( CurrentBufferPosition >= SERIAL_INPUT_BUFFER_MAX ) CurrentBufferPosition = (SERIAL_INPUT_BUFFER_MAX - 1);  // Capture Overflows.
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
  

  // Run Motor Full Command - FM
  if( (strcmp( CommandHeader, "FM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_FULL;
  }

  // Run Motor Half Command - HM
  if( (strcmp( CommandHeader, "HM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_HALF;
  }  

  // Stop Motor Command - SM
  if( (strcmp( CommandHeader, "SM" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoRev = true;
    CommandRev = COMMAND_REV_NONE;
  }
 

  // Full Power Command - FP
  if( (strcmp( CommandHeader, "FP" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpeedFull = constrain( atoi( IntValue ), 30, 100 );
    TimeLastMotorSpeedChanged = millis();
    EEPROM.write( ADDR_HIGHMOTORSPEED, MotorSpeedFull );
  }

  // Half Power Command - HP
  if( (strcmp( CommandHeader, "HP" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    char IntValue[4] = { SerialInputBuffer[3], SerialInputBuffer[4], SerialInputBuffer[5], 0 };
    MotorSpeedHalf = constrain( atoi( IntValue ), 30, 100 );
    TimeLastMotorSpeedChanged = millis();
    EEPROM.write( ADDR_LOWMOTORSPEED, MotorSpeedHalf );
  }

  // Query Device Command - QD
  if( strcmp( CommandHeader, "QD" ) == 0 )
  {
    Serial.println( F("#R2-OK$") );
    Serial1.println( F("#R2-OK$") );
  }

  // Get Info Command - GI
  if( strcmp( CommandHeader, "GI" ) == 0 )
  {
    /*
     * Format:
     * 
     * GI-FP-HP-PSI-MS-DIM-DF
     * #GI-000-000-0-00-00-00000$
     * Len = 26
     * 
     */
    sprintf( SerialInputBuffer, "#GI-%03d-%03d-%d-%02d-%02d-%05d$", MotorSpeedFull, MotorSpeedHalf, PusherSpeedIndex, MagSize, DartsInMag, TotalDartsFired );
    
    Serial.print( SerialInputBuffer );
    Serial1.print( SerialInputBuffer );
  }

  // Display Settings - DS
  if( strcmp( CommandHeader, "DS" ) == 0 )
  {
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );
    Serial.println( F("Blaster Settings:") );
    Serial1.println( F("Blaster Settings:") );
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );    

    Serial.print( F("Full Trigger Power = ") );
    Serial1.print( F("Full Trigger Power = ") );
    Serial.println( MotorSpeedFull );
    Serial1.println( MotorSpeedFull );    
    Serial.println( F("Change with #FP-xxx$  (xxx = 030 - 100)\n") );
    Serial1.println( F("Change with #FP-xxx$  (xxx = 030 - 100)\n") );

    Serial.print( F("Half Trigger Power = ") );
    Serial1.print( F("Half Trigger Power = ") );
    Serial.println( MotorSpeedHalf );
    Serial1.println( MotorSpeedHalf );     
    Serial.println( F("Change with #HP-xxx$  (xxx = 030 - 100)\n") );
    Serial1.println( F("Change with #HP-xxx$  (xxx = 030 - 100)\n") );

    Serial.print( F("Burst Size = ") );
    Serial1.print( F("Burst Size = ") );
    Serial.println( BurstSize );
    Serial1.println( BurstSize );    
    Serial.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );
    Serial1.println( F("Change with #BS-xx$  (xx = 01 - 99)\n") );

    Serial.print( F("Mag Size = ") );
    Serial1.print( F("Mag Size = ") );
    Serial.println( MagSize );
    Serial1.println( MagSize );    
    Serial.println( F("Change with #DM-xx$  (xx = 06 - 35)\n") );
    Serial1.println( F("Change with #DM-xx$  (xx = 06 - 35)\n") );

    Serial.print( F("Pusher Speed = ") );
    Serial1.print( F("Pusher Speed = ") );    
    Serial.println( PusherSpeedIndex );
    Serial1.println( PusherSpeedIndex );    
    Serial.println( F("Change with #PS-x$  (x = 0 - 2; 0 = Low, 1 = Med, 2 = High)\n") );
    Serial1.println( F("Change with #PS-x$  (x = 0 - 2; 0 = Low, 1 = Med, 2 = High)\n") );
    Serial.println( F("--------------------\n") );
    Serial1.println( F("--------------------\n") );

    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") ); 
    Serial.println( F("Blaster Status:") );
    Serial1.println( F("Blaster Status:") );
    Serial.println( F("--------------------") );
    Serial1.println( F("--------------------") );

    Serial.print( F("Battery Voltage = ") );
    Serial1.print( F("Battery Voltage = ") );
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );
    Serial1.println( VoltBuffer );   

    Serial.print( F("Motor Ramp Up Rate = ") );
    Serial1.print( F("Motor Ramp Up Rate = ") );
    Serial.println( MotorRampUpPerMS );
    Serial1.println( MotorRampUpPerMS );  
    
    Serial.print( F("Motor Ramp Down Rate = ") );
    Serial1.print( F("Motor Ramp Down Rate = ") );
    Serial.println( MotorRampDownPerMS );
    Serial1.println( MotorRampDownPerMS );  

    Serial.print( F("Battery S = ") );
    Serial1.print( F("Battery S = ") );
    Serial.println( BatteryS );
    Serial1.println( BatteryS );

    Serial.print( F("System Mode = ") );
    Serial1.print( F("System Mode = ") );
    Serial.println( SystemMode );
    Serial1.println( SystemMode );
    
    Serial.print( F("Full Trigger State = ") );
    Serial1.print( F("Full Trigger State = ") );
    Serial.println( FireFullTriggerPressed );
    Serial1.println( FireFullTriggerPressed );

    Serial.print( F("Rev Trigger State = ") );
    Serial1.print( F("Rev Trigger State = ") );
    Serial.println( RevTriggerPressed );
    Serial1.println( RevTriggerPressed );

    Serial.print( F("Darts In Mag = ") );
    Serial1.print( F("Darts In Mag = ") );
    Serial.println( DartsInMag );
    Serial1.println( DartsInMag );  

    Serial.print( F("Darts Fired This Session = ") );
    Serial1.print( F("Darts Fired This Session = ") );
    Serial.println( TotalDartsFired );
    Serial1.println( TotalDartsFired );

    Serial.print( F("Total Darts Fired = ") );
    Serial1.print( F("Total Darts Fired = ") );
    Serial.println( LifetimeTotalDartsFired );
    Serial1.println( LifetimeTotalDartsFired );    

    Serial.println( F("--------------------\n") );
    Serial1.println( F("--------------------\n") );               
  } 

  // Single Fire Full Command - SF
  if( (strcmp( CommandHeader, "SF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }

  // Single Fire Half Command - SH
  if( (strcmp( CommandHeader, "SH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    CurrentFireMode = FIRE_MODE_SINGLE;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
  }

  // Query Voltage Command - QV
  if( strcmp( CommandHeader, "QV" ) == 0 )
  {
    char VoltBuffer[5];
    sprintf( VoltBuffer, "%3d", (int)(BatteryCurrentVoltage * 10) );
    VoltBuffer[4] = 0;
    VoltBuffer[3] = VoltBuffer[2];
    VoltBuffer[2] = '.';
    Serial.println( VoltBuffer );
    Serial1.println( VoltBuffer );
  }   

  // Burst Fire Full Command - BF
  if( (strcmp( CommandHeader, "BF" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_FULL;
  }  

  // Burst Fire Full Command - BH
  if( (strcmp( CommandHeader, "BH" ) == 0) && (SystemMode == MODE_NORMAL) )
  {
    AutoFire = true;
    if( !HasSavedMode )
    {
      HasSavedMode = true;
      SavedMode = CurrentFireMode;
      SavedBurstSize = BurstSize;
    }
    char IntValue[3] = { SerialInputBuffer[3], SerialInputBuffer[4], 0 };
    BurstSize = constrain( atoi( IntValue ), 1, 99 );
    CurrentFireMode = FIRE_MODE_BURST;
    AutoFireMotorSpeed = COMMAND_REV_HALF;
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
 * Process the manual commands leading to motor reving
 * 
 * Logic:
 * If AutoRev is being performed, disconnect it when the half trigger is pulled.
 * We are looking for the following events: 
 * If the Half Trigger is pressed, Rev to Speed A
 * If the Rev Trigger is pressed, and the Half Trigger is also pressed, Rev to Speed B
 * If the Rev Trigger is pressed, but the Half Trigger is not, then ignore the command.
 * 
 */
void ProcessRevCommand()
{
  
  static bool PreviousRevTriggerPressed = false; // Keep track of the human input

  if( !(SystemMode == MODE_NORMAL) ) // Spin the motors down when something out of the ordinary happens.
  {
     CommandRev = COMMAND_REV_NONE;
     AutoRev = false;
     return;
  }

  if( (PreviousRevTriggerPressed != RevTriggerPressed) && (SystemMode == MODE_NORMAL) )
  {
    // Human has taken control - disengage autopilot but only when not in config mode
    PreviousRevTriggerPressed = RevTriggerPressed;
    AutoRev = false;
  }

  if( !AutoRev )
  {
    if( RevTriggerPressed )
    {
      CommandRev = COMMAND_REV_FULL;
    }
    else
    {
      CommandRev = COMMAND_REV_NONE;
    }
  }
  // Else the computer is controlling, and the current rev trigger state is ignored. Autopilot will adjust CommandRev
  
}

// Update the motors with the new speed
void ProcessMainMotors()
{
  static int PreviousMotorSpeed = MinMotorSpeed;

  if( PreviousMotorSpeed != CurrentMotorSpeed ) 
  { 
    // Debugging output
    //Serial.println(CurrentMotorSpeed);

    // Use this for Servo Library
    if( CurrentMotorSpeed > MOTOR_MAX_SPEED )
      UpdatePWM( MOTOR_MAX_SPEED );
    else
      UpdatePWM( CurrentMotorSpeed );

    PreviousMotorSpeed = CurrentMotorSpeed;
  }
}

/*
 * Calculate the desired motor speed
 */
void ProcessMotorSpeed()
{
  // Don't do anything if the motor is already running at the desired speed.
  if( CurrentMotorSpeed == TargetMotorSpeed )
  {
    return;
  }

  unsigned long CurrentTime = millis(); // Need a base time to calcualte from
  unsigned long MSElapsed = CurrentTime - TimeLastMotorSpeedChanged;
  if( MSElapsed == 0 ) // No meaningful time has elapsed, so speed will not change
  {
    return;
  }
  if( CurrentMotorSpeed < TargetMotorSpeed )
  {
    long SpeedDelta = (MSElapsed * MotorRampUpPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.    
    int NewMotorSpeed = CurrentMotorSpeed + SpeedDelta; // Calclate the new motor speed..  

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed + 10 >= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
  if( CurrentMotorSpeed > TargetMotorSpeed )
  {
    //Serial.println( MSElapsed );
    long SpeedDelta = (MSElapsed * MotorRampDownPerMS / 1000);
    if( SpeedDelta < 1 ) return; // Not enough cycles have passed to make an appreciable difference to speed.
    int NewMotorSpeed = CurrentMotorSpeed - SpeedDelta; // Calclate the new motor speed..

    // If it's within 1% (which is 10) of target, then just set it
    if( NewMotorSpeed - 10 <= TargetMotorSpeed )
    {
      NewMotorSpeed = TargetMotorSpeed;
    }

    TimeLastMotorSpeedChanged = CurrentTime;
    CurrentMotorSpeed = NewMotorSpeed;
  }
}

// We need to set the Target Motor Speed here.
void ProcessSpeedControl()
{
  static byte LastSetMaxSpeed = 100;

  if( CommandRev == COMMAND_REV_HALF ) SetMaxSpeed = MotorSpeedHalf;
  if( CommandRev == COMMAND_REV_FULL ) SetMaxSpeed = MotorSpeedFull;
  if( CommandRev == COMMAND_REV_NONE ) SetMaxSpeed = 0;

  if( LastSetMaxSpeed == SetMaxSpeed ) return; // Speed hasn't changed

  if( CommandRev > COMMAND_REV_NONE ) 
  {
    SetMaxSpeed = constrain( SetMaxSpeed, 30, 100 ); // Constrain between 30% and 100%
  }
  
  TargetMotorSpeed = map( SetMaxSpeed, 0, 100, MinMotorSpeed, MaxMotorSpeed );  // Find out our new target speed.

  LastSetMaxSpeed = SetMaxSpeed;

  Serial.print( F("New max speed % = ") );
  Serial.println( SetMaxSpeed );

  Serial.print( F("New target speed = ") );
  Serial.println( TargetMotorSpeed );
}

void ProcessFiring()
{
  if( !(SystemMode == MODE_NORMAL) ) // Finish off the stroke unless in running or in ROF config mode
  {
    ShotsToFire = 0;
    if( ProcessingFireMode == FIRE_MODE_AUTO_LASTSHOT )
      ProcessingFireMode = FIRE_MODE_IDLE;
    return;
  }

  static unsigned long InitiatedAutoFire = 0;
  if( AutoFire )
  {
    if( (InitiatedAutoFire == 0) && (CommandRev == COMMAND_REV_NONE) ) // Started auto fire process. Start spinning the motors
    {
      InitiatedAutoFire = millis();
      AutoRev = true;
      CommandRev = AutoFireMotorSpeed;
      return;
    }
    else if( InitiatedAutoFire == 0 )
    {
      InitiatedAutoFire = millis();
      return;      
    }
    if( (millis() - InitiatedAutoFire) < (MOTOR_SPINUP_LAG) ) // Wait for the lag
    {
      return;
    }
    RequestShot = true;
  }
  else
  {
    InitiatedAutoFire = 0;
  }
    

  if( (CommandRev == COMMAND_REV_NONE) && (SystemMode == MODE_NORMAL) && RequestShot ) // Trigger was pushed. 
  {
    AutoFire = true; // Actually, do nothing now except for indicate that we need to be 
    AutoFireMotorSpeed = COMMAND_REV_FULL;
    InitiatedAutoFire = 0;
    return;
  }

  // Requesting Shot while we were doing nothing special
  if( RequestShot && (ProcessingFireMode == FIRE_MODE_IDLE) )
  {
    ProcessingFireMode = CurrentFireMode;
    switch( ProcessingFireMode )
    {
      case FIRE_MODE_SINGLE:
        ShotsToFire = 1; // Add another shot to the queue
        PusherROF = GetPusherSpeed( ROF_SINGLE );
        ExecuteFiring = true;
        break;
      case FIRE_MODE_BURST:
        ShotsToFire = BurstSize; // Set the burst size
        PusherROF = GetPusherSpeed( PusherSpeedIndex );
        ExecuteFiring = true;
        break;        
      case FIRE_MODE_AUTO:
        ShotsToFire = 9999; // Set to something unreasonably high
        PusherROF = GetPusherSpeed( PusherSpeedIndex );
        ExecuteFiring = true;
        break;        
    }

    if( ShotsToFire > 0 )
    {
      StartPusher();
    }
  }
  else if( RequestAutoStop && (ProcessingFireMode == FIRE_MODE_AUTO) ) // Requesting Stop while firing in Full Auto 
  {
    ProcessingFireMode = FIRE_MODE_AUTO_LASTSHOT;
    PusherROF = GetPusherSpeed( ROF_LASTSHOT );
    ExecuteFiring = true;
    ShotsToFire = 0;
  }
}

// Use this to manage requests to handle the pusher state.
void ProcessPusher()
{
  // Logic:
  // 1 - Do we need to change? If so, initiate transition
  // 2 - Are we in transition? If so, write digital 0 to both fets. Keep doing that
  // 3 - Are we out of transition? If so, 
  // 4.a  - Are we wanting to run? Are we already running or just out of transition?
  //        If so, turn on pusher fet, ensure brake fet is off
  //        Else turn fets off, and start transition timer
  // 4.b  - Are we wanting to stop? Are we already stoppedor just out of transition?
  //        If so, turn off pusher fet, and turn on brake fet
  //        Else turn fets off and start transition timer


  static bool LastPusherRequest = true;
  static int SelectedTransitionTime = 100;

  // Step 1 - initiate transition
  if( LastPusherRequest != PusherRequest )
  {
    CurrentPusherStatus = PusherTransition;
    if( PusherRequest )
    {
      SelectedTransitionTime = PusherOnTransitionTime;
    }
    else
    {
      SelectedTransitionTime = PusherOffTransitionTime;
    }
    PusherTransitionStart = millis();
    LastPusherRequest = PusherRequest;
    Serial.println( "Pusher FET now in Transition" );
    return;
  }

  // Step 2 - wait for transition to complete
  if( CurrentPusherStatus == PusherTransition )
  {
    digitalWrite( PIN_PUSHER_RUN, LOW );
    PusherPWMFETOn = false;
    digitalWrite( PIN_PUSHER_STOP, LOW );
    PusherBrakeFETOn = false;
    if( millis() - PusherTransitionStart <= SelectedTransitionTime )
      return;

    // We are now out of transition
    if( PusherRequest )
    {
      CurrentPusherStatus = PusherRun;
      Serial.println( "Pusher FET now in RUN" );
    }
    else
    {
      CurrentPusherStatus = PusherStop;
      Serial.println( "Pusher FET now in STOP" );
    }
    
    return;
  }
  
  if( CurrentPusherStatus == PusherRun )
  {
    // Check to make sure brake fet is not on
    if( PusherBrakeFETOn )
    {
      Serial.println( F( "** ERROR ** Pusher Run Status with Brake FET on!!!" ) );
      return;
    }
    if( JamDetected )
    {
      digitalWrite( PIN_PUSHER_RUN, LOW );
      return;
    }


    if( (millis() - TimeLastPusherResetOrActivated) > PusherMaxCycleTime )
    {
      // Jam detected
      digitalWrite( PIN_PUSHER_RUN, LOW );
      JamDetected = true;
      return;
    }
    else
    {
      JamDetected = false;
    }

    if( PusherROF == 100 )
    {
      // Digital write HIGH for 100%
      digitalWrite( PIN_PUSHER_RUN, HIGH );
      //Serial.println( 255 );
    }
    else if( PusherROF == 0 )
    {
      // Just in case we want 0%
      digitalWrite( PIN_PUSHER_RUN, LOW );
      //Serial.println( 0 );
    }
    else
    {
      // PWM Write
      int PWM = map( PusherROF, 0, 100, 0, 255 );
      analogWrite( PIN_PUSHER_RUN, PWM );
      //Serial.println( PWM );
    }
    PusherPWMFETOn = true;
    
    digitalWrite( PIN_PUSHER_STOP, LOW );
    PusherBrakeFETOn = false;

    return;
  }

  if( CurrentPusherStatus == PusherStop )
  {
    // Check to make sure motor fet is not on
    if( PusherPWMFETOn )
    {
      Serial.println( F( "** ERROR ** Pusher Stop Status with PWM FET on!!!" ) );
      return;
    }

    digitalWrite( PIN_PUSHER_RUN, LOW );
    PusherPWMFETOn = false;
    
    digitalWrite( PIN_PUSHER_STOP, HIGH );
    PusherBrakeFETOn = true;

    return;
  }
}

void StartPusher()
{  
  PusherStopping = false;
  PusherRequest = true;
  TimeLastPusherResetOrActivated = millis();
}

void StopPusher()
{
  Serial.println( F("Pusher Stopped!!") );
  JamDetected = false;
  PusherRequest = false;
}

/*
 * Process When the Pusher Returns Home
 */
void ProcessPusherReturn()
{
  static bool LastPusherResetPressed = true;


  if( PusherResetBounce.fell() )
  {
    if( CommandRev == COMMAND_REV_NONE )
    {
      ShotsToFire = 0;
      Serial.println( "EMERGENCY HALT" );
    } 
    if( ShotsToFire > 1 )
    {
      ShotsToFire --; // Decrease the number of darts to fire.
      if( DartsInMag > 0 )
      {
        DartsInMag --; 
        TotalDartsFired ++;
        LifetimeTotalDartsFired ++;
      }
      TimeLastPusherResetOrActivated = millis();
      Serial.print( F("Darts remaining: ") );
      Serial.println( ShotsToFire );
    }
    else
    {
      ShotsToFire = 0;
      if( (ShotsToFire <= 1) && (ProcessingFireMode != FIRE_MODE_SINGLE ) )
      {
        Serial.println( "Slow down pusher" );
        PusherROF = GetPusherSpeed( ROF_LASTSHOT );      
      }
      if( DartsInMag > 0 )
      {
        DartsInMag --; // Count down any misfires though  
        TotalDartsFired ++;
        LifetimeTotalDartsFired ++;
      }
      Serial.println( F("Halt pusher") );
      FiringLastShot = false;
      PusherStopping = true;
      StopPusher(); 

      ExecuteFiring = false;
      if( AutoFire )
      {
        AutoFire = false;
        if( HasSavedMode )
        {
          BurstSize = SavedBurstSize;
          CurrentFireMode = SavedMode;
          HasSavedMode = false;
        }
        RequestShot = false;
        AutoRev = false;
        CommandRev = COMMAND_REV_NONE;
        ProcessingFireMode = FIRE_MODE_IDLE;
      }
      else
      {
        ProcessingFireMode = FIRE_MODE_IDLE; 
        RequestShot = false;      
        if( HasSavedMode )
        {
          BurstSize = SavedBurstSize;
          CurrentFireMode = SavedMode;
          HasSavedMode = false;
        }
      }
    }
  }
  if( PusherResetBounce.rose() )
  {
    if( (ShotsToFire <= 1) && (ExecuteFiring) )
    {
      FiringLastShot = true;
      PusherROF = GetPusherSpeed( ROF_LASTSHOT );
      Serial.println( "Running last shot now!" );
    }
    else
    {
      Serial.println( "Not Last!" );
      FiringLastShot = false;
    }
  }
}

// Keep tabs on the battery.
void ProcessBatteryMonitor()
{
  
  // Only count one in 10 run-through cycles
  static byte RunNumber = 0;
  RunNumber++;
  if( RunNumber <= 250 ) // Only read once every 300 cycles.. For performance reasons.
    return;
  RunNumber = 0;
  
  #define NUM_SAMPLES 20
  static byte CollectedSamples = 0;
  static float SampleAverage = 0;
  float SensorValue = analogRead( PIN_BATTERY_MONITOR );
  if( CollectedSamples < NUM_SAMPLES )
  {
    CollectedSamples ++;
    SampleAverage += SensorValue;
  }
  else
  {
    BatteryCurrentVoltage = (((float)SampleAverage / (float)CollectedSamples * 5.0)  / 1024.0 * (float)((47.0 + 10.0) / 10.0)) + BATTERY_CALFACTOR;  // Voltage dividor - 47k and 10k
    if( BatteryCurrentVoltage < BatteryMinVoltage )
    {
      if( BatteryCurrentVoltage > 1.6 ) // If the current voltage is 0, we are probably debugging
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
    BatteryPercent = map( (int)(BatteryCurrentVoltage * 10), (int)(BatteryMinVoltage * 10), (int)(BatteryMaxVoltage * 10), 1, 100 );
    //Serial.print( ("BatteryVoltage = ") );
    //Serial.println( BatteryCurrentVoltage );
    CollectedSamples = 0;
    SampleAverage = 0;
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

void ProcessDisplay()
{
  static int LastSystemMode = MODE_NORMAL;
  bool ClearScreen = false;
  
  if( LastSystemMode != SystemMode )
  {
    ClearScreen = true;
    LastSystemMode = SystemMode;
  }

  Display_ScreenHeader( ClearScreen );
  switch( SystemMode )
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
    case MODE_PUSHER_JAM:
      Display_Jam( ClearScreen );
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

void Display_Jam( bool ClearScreen )
{
  if( ClearScreen )
  {
    Serial.println( F("JAM DETECTED!!") );
    Serial.println( BatteryCurrentVoltage );

    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);
    display.print( F("################\n") );
    display.print( F("# JAM DETECTED #\n") );
    display.print( F("################") ); 
  }
}

void Display_Normal( bool ClearScreen )
{
  char Buffer[4];
  static int LastDartsInMag = 99;
  static byte LastCurrentFireMode = 99;
  static byte LastPusherSpeedIndex = 99;
  static byte LastMotorSpeedFull = 199;
  static byte LastMotorSpeedHalf = 199;
  static byte LastBurstSize = 99;
  if( ClearScreen )
  {
    Serial.println( F("NORMAL MODE!!") );
  }
  if( ClearScreen || (CurrentFireMode != LastCurrentFireMode) || (BurstSize != LastBurstSize) )
  {  
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 2);
    if( CurrentFireMode == FIRE_MODE_SINGLE )
    {
      display.print( F("Single    ") );
    }
    else if( CurrentFireMode == FIRE_MODE_AUTO )
    {
      display.print( F("Full Auto ") );
    }
    else
    {
      sprintf( Buffer, "%2d", BurstSize );
      display.print( F("Burst: ") );
      display.print( Buffer );
    }
  }
  if( ClearScreen || (PusherSpeedIndex != LastPusherSpeedIndex) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 4);
    display.print( F("ROF: ") );
    if( PusherSpeedIndex == ROF_SLOW )
      display.print( F("L") );
    else if( PusherSpeedIndex == ROF_MEDIUM )
      display.print( F("M") );
    else
      display.print( F("H") );
  }
  if( ClearScreen || (MotorSpeedFull != LastMotorSpeedFull) || (MotorSpeedHalf != LastMotorSpeedHalf) )
  {
    display.setFont(ZevvPeep8x16);
    display.setCursor(0, 6);
    display.print( F("Pwr:") );
    sprintf( Buffer, "%3d", MotorSpeedFull );
    display.print( Buffer );
    display.print( "/" );
    sprintf( Buffer, "%3d", MotorSpeedHalf );
    display.print( Buffer );
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
  LastBurstSize = BurstSize;
  LastMotorSpeedHalf = MotorSpeedHalf;
  LastMotorSpeedFull = MotorSpeedFull;
  LastCurrentFireMode = CurrentFireMode;
}

void Display_ScreenHeader( bool ClearScreen )
{
  static float LastBatteryVoltage = 99.0;
  static unsigned int LastTotalDartsFired = 9999;
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
    display.print( "-> " );
    display.print( Buffer );      
  }

  LastBatteryVoltage = BatteryCurrentVoltage;
  LastTotalDartsFired = TotalDartsFired;
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

    EEPROM.put( ADDR_TDF, LifetimeTotalDartsFired ); // Only save one time.
  }

  JamDetected = false;

  PusherResetBounce.update();
  if( !(PusherResetBounce.read() == LOW) ) // Currently not in reset position, or close enough
  {
    PusherROF = GetPusherSpeed( ROF_RETRACT );
    StartPusher();
  }
  else
  {
    if( PusherRequest ) StopPusher(); 
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

    char BurstBuffer[4];
    sprintf( BurstBuffer, "%2d", BurstSize );
    char MagBuffer[4];
    sprintf( MagBuffer, "%2d", MagSize );    
    char HighSpeedBuffer[5];
    sprintf( HighSpeedBuffer, "%3d", MotorSpeedFull );    
    char LowSpeedBuffer[5];
    sprintf( LowSpeedBuffer, "%3d", MotorSpeedHalf );    
    char TDFBuffer[7];
    sprintf( TDFBuffer, "%6d", LifetimeTotalDartsFired );    
       
    switch( MenuItem )
    {
      case 0:
        display.print( F("> Full Pwr: ") );
        display.print( MotorSpeedFull );
        display.print( F("% \n") );
        display.print( F("  Half Pwr: ") );
        display.print( MotorSpeedHalf );
        display.print( F("% \n") );
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      ") );
        break;
      case 1:
        display.print( F("  Full Pwr: ") );
        display.print( MotorSpeedFull );
        display.print( F("% \n") );
        display.print( F("> Half Pwr: ") );
        display.print( MotorSpeedHalf );
        display.print( F("% \n") );
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("      ") );
        break;
      case 2:
        display.print( F("  Half Pwr: ") );
        display.print( MotorSpeedHalf );
        display.print( F("%\n") );
        display.print( F("> ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("       \n") );
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   ") ); 
        break;
      case 3:
        display.print( F("  ROF: ") );
        if( PusherSpeedIndex == 0 )
          display.print( F("L") );
        else if( PusherSpeedIndex == 1 )
          display.print( F("M") );
        else
          display.print( F("H") );
        display.print( F("       \n") );
        display.print( F("> Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   \n") ); 
        display.print( F("  Mag Size: ") );
        display.print( MagBuffer );
        display.print( F("   ") ); 
        break;
      case 4:
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   \n") ); 
        display.print( F("> Mag Size: ") );
        display.print( MagBuffer );
        display.print( F("   \n") );
        display.print( F("  TDF: ") );
        display.print( TDFBuffer );
        display.print( F(" ") );          
        break;      
      default:          
        display.print( F("  Burst: ") );
        display.print( BurstBuffer );
        display.print( F("   \n") ); 
        display.print( F("  Mag Size: ") );
        display.print( MagBuffer );
        display.print( F("   \n") ); 
        display.print( F("> TDF: ") );
        display.print( TDFBuffer );
        display.print( F(" ") ); 
        break;
    }

    LastMenuItem = MenuItem;
  }
  
  if( RevTriggerBounce.fell() )
  {
    MenuItem++;
    if( MenuItem > 5 )
      MenuItem = 0;
  }
   

  if( FireFullTriggerBounce.fell() )
  {
    switch( MenuItem )
    {
      case 0:
        MotorSpeedFull -= 5;
        if( MotorSpeedFull < 30 )
          MotorSpeedFull = 100;
        EEPROM.write( ADDR_HIGHMOTORSPEED, MotorSpeedFull );
        break;
      case 1:
        MotorSpeedHalf -= 5;
        if( MotorSpeedHalf < 30 )
          MotorSpeedHalf = 100;
        EEPROM.write( ADDR_LOWMOTORSPEED, MotorSpeedHalf );
        break;
      case 2:
        PusherSpeedIndex ++;
        if( PusherSpeedIndex > 2 )
          PusherSpeedIndex = 0;
        EEPROM.write( ADDR_ROF, PusherSpeedIndex );  
        break;
      case 3:
        BurstSize ++;
        if( BurstSize > MagSize )
          BurstSize = 1;
        EEPROM.write( ADDR_BURST, BurstSize );
        break;
      case 4:
        MagSize ++;
        if( MagSize > 35 )
          MagSize = 6;
        EEPROM.write( ADDR_MAGSIZE, MagSize );
        break;
      default:
        break;
    }
    LastMenuItem = 99;
  }
    
}
