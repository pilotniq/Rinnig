/* 
 Keyboard test
 
 For the Arduino Leonardo or Micro
 
 Reads a byte from the serial port, sends a keystroke back.
 The sent keystroke is one higher than what's received, e.g.
 if you send a, you get b, send A you get B, and so forth.
 
 The circuit:
 * none
 
 created 21 Oct 2011
 modified 27 Mar 2012
 by Tom Igoe
 
This example code is in the public domain.
 
 http://www.arduino.cc/en/Tutorial/KeyboardSerial
 */

#include <SPI.h>

// The shortest permitted time between button state changes.
#define DEBOUNCE_TIME_MILLIS 40
#define SERIAL_DEBUG 1

#define PLUNGER_FILTER_LENGTH 4
#define PLUNGER_DEADBAND 16

const int PIN_PLUNGER = 0;
const int PIN_LEFT_FLIPPER = 4;
const int PIN_RIGHT_FLIPPER = 5;
const int PIN_MENU_BUTTON = 7;
const int PIN_START_BUTTON = 6;
const int PIN_START_LED = 10;
const int PIN_ACC_CS = 8;

typedef struct sButton 
{
  int pin;
  char key;
  int state;
  long earliestNextStateChangeTime;
} sButton, *Button;

sButton buttons[] = { { PIN_LEFT_FLIPPER, 'l', HIGH, 0 },
                      { PIN_RIGHT_FLIPPER, 'r', HIGH, 0 },
                      { PIN_START_BUTTON, '1', HIGH, 0 },
                      { PIN_MENU_BUTTON, KEY_ESC, HIGH, 0 } };
1
int plungerState = 0;
long idleTicks = 0;
int plungerWait = 0;
int prevPlungerValue = 0;
long plungerFilter = 0;

void setup() {
  adxl345_init( PIN_ACC_CS );
  adxl345_calibrate();
  
  for( int i = 0; i < sizeof( buttons ) / sizeof( sButton ); i++ )
    pinMode( buttons[i].pin, INPUT_PULLUP );
  
  pinMode( PIN_START_LED, OUTPUT );
  analogWrite( PIN_START_LED, 4 );
  
  // initialize control over the keyboard:
  Keyboard.begin();
  Gamepad.begin();
#if SERIAL_DEBUG  
  Serial.begin(9600);
#endif
}

void loop() 
{
  static unsigned long lastAccSampleTime = 0;
  int plungerValue;
  int accValues[3];
  unsigned long now;
  
  for( int i = 0; i < sizeof( buttons ) / sizeof( sButton ); i ++ )
  {
    int state;
    long now = millis();
    
    state = digitalRead( buttons[i].pin );
    if( (state != buttons[ i ].state) && 
        (now >= buttons[i].earliestNextStateChangeTime) )
    {
      // check if sufficient time has passed since last state change. 10 ms?
      if( state == HIGH )
        Keyboard.release( buttons[i].key );
      else
        Keyboard.press( buttons[i].key );
      
      buttons[i].state = state;
      buttons[i].earliestNextStateChangeTime = now + DEBOUNCE_TIME_MILLIS;
      
      idleTicks = 0;
    }
  } 

  // THe plunger sensor returns new values about once every 20 ms.
  if( plungerWait >= 7 )
  {
    int axisValue;
    
    plungerValue = analogRead( PIN_PLUNGER );
    axisValue = 361 - plungerValue;
    if( axisValue > 127 )
      axisValue = 127;
    
    plungerFilter = plungerFilter * (PLUNGER_FILTER_LENGTH - 1) / 
                       PLUNGER_FILTER_LENGTH + axisValue;
    int newPlungerValue = (plungerFilter / PLUNGER_FILTER_LENGTH);
    
    if( (newPlungerValue <= PLUNGER_DEADBAND) && 
        (newPlungerValue >= -PLUNGER_DEADBAND) )
      newPlungerValue = 0;
      
    if( newPlungerValue != prevPlungerValue )
    {
      Gamepad.zAxis( newPlungerValue );
      Gamepad.write();
      prevPlungerValue = newPlungerValue;

#if SERIAL_DEBUG
    Serial.print( millis() );
    Serial.print( '\t' );
    Serial.print( newPlungerValue );
    Serial.print( '\t' );
    Serial.println( plungerValue );
#endif
    }
 
    plungerWait = 0;
  }
  else
    plungerWait++;
  
  now = millis();
  if( (now - lastAccSampleTime) > 10 )
  {
    // Read from accellerometer
    adxl345_sample( accValues );
    lastAccSampleTime = now;
    
    Gamepad.xAxis( accValues[0] * 512 );
    Gamepad.yAxis( accValues[1] * 512 );
    
    // adjust offsets slowly for return to zero behaviour

    Gamepad.write();
  }
  delay( 2 );
  
  idleTicks++;
}

/*
 * Start of ADXL345 Accellerometer functions
 */
 //This is a list of some of the registers available on the ADXL345.
//To learn more about these and the rest of the registers on the ADXL345, read the datasheet!
char OFSX = 0x1E;  // Offset X
char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAX1 = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAY1 = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAZ1 = 0x37;	//Z-Axis Data 1

//These variables will be used to hold the x,y and z axis accelerometer values.
// int x,y,z;
int adxl345_offset[3];

static int adxl_csPin;

void adxl345_init( int pinNumber )
{
  adxl_csPin = pinNumber;
  
  //Initiate an SPI communication instance.
  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode( adxl_csPin, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite( adxl_csPin, HIGH);
  
  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  adxl345_writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  adxl345_writeRegister(POWER_CTL, 0x08);  //Measurement mode  
}

void adxl345_calibrate()
{
  int values[3];
  long sum[3] = { 0, 0, 0 };
  int i;
  
  for( i = 0; i < 10; i++ )
  {
    adxl345_sampleRaw( values );
    // print3Vector( values );
    sum[0] += ((long) values[0]);
    sum[1] += ((long) values[1]);
    sum[2] += ((long) values[2]);
    // print3LongVector( sum );
    
    delay(10);
  }
  
  // Serial.print( "sums=" );
  // print3Vector( sum );
  
  adxl345_offset[0] = sum[0] / 10;
  adxl345_offset[1] = sum[1] / 10;
  adxl345_offset[2] = sum[2] / 10;

  // Serial.print( "offsets=" );
  // print3Vector( offset );
}

void adxl345_sample( int *values )
{
  static int sampleCounter = 0;
  
  adxl345_sampleRaw( values );

  for( int i = 0; i < 3; i++ )
  {
    values[i] = values[i] - adxl345_offset[i];

    /* slow return to zero feature */
    if( (sampleCounter & 0x3f) == 0 )
    {
      if( values[i] > 5 )
        adxl345_offset[i]++;
      if( values[i] < -5 )
        adxl345_offset[i]--;
    }
  }
  sampleCounter++;
}

void adxl345_sampleRaw( int *values )
{
  char rawValues[6];
  
  adxl345_readRegister(DATAX0, 6, rawValues );

  //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  //The X value is stored in values[0] and values[1].
  values[0] = ((int)rawValues[1]<<8)|(int)rawValues[0];
  //The Y value is stored in values[2] and values[3].
  values[1] = ((int)rawValues[3]<<8)|(int)rawValues[2];
  //The Z value is stored in values[4] and values[5].
  values[2] = ((int)rawValues[5]<<8)|(int)rawValues[4];
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void adxl345_writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(adxl_csPin, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(adxl_csPin, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void adxl345_readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;
  
  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(adxl_csPin, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(adxl_csPin, HIGH);
}

