/** Arduino library for PWM and servo driver.
*/

/* ************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface.

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "ZPCA9685.h"

#include <Wire.h>

#include "PinExtender.h"

   // Register addresses from data sheet
#define ZPCA9685_MODE1_REG           (byte)0x00
#define ZPCA9685_MODE2_REG           (byte)0x01
#define ZPCA9685_SUBADR1_REG         (byte)0x02
#define ZPCA9685_SUBADR2_REG         (byte)0x03
#define ZPCA9685_SUBADR3_REG         (byte)0x04
#define ZPCA9685_ALLCALL_REG         (byte)0x05
#define ZPCA9685_LED0_REG            (byte)0x06 // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define ZPCA9685_PRESCALE_REG        (byte)0xFE
#define ZPCA9685_ALLLED_REG          (byte)0xFA
#define ZPCA9685_ADDRESS_MASK        0x40
#define ZPCA9685_ADDRESS             0x40

#define ZPCA9685_SUBADR1 0x2
#define ZPCA9685_SUBADR2 0x3
#define ZPCA9685_SUBADR3 0x4

#define ZPCA9685_MODE1 0x0
#define ZPCA9685_MODE2 0x1
#define ZPCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

 #define NO_CHANNEL 0xfe
 #define ZPCA9685_PWM_FULL            (uint16_t)0x01000 // Special value for full on/full off LEDx modes

// Mode1 register pin layout
#define ZPCA9685_MODE_RESTART        (byte)0x80
#define ZPCA9685_MODE_EXTCLK         (byte)0x40
#define ZPCA9685_MODE_AUTOINC        (byte)0x20
#define ZPCA9685_MODE_SLEEP          (byte)0x10
#define ZPCA9685_MODE_SUBADR1        (byte)0x08
#define ZPCA9685_MODE_SUBADR2        (byte)0x04
#define ZPCA9685_MODE_SUBADR3        (byte)0x02
#define ZPCA9685_MODE_ALLCALL        (byte)0x01


 #define ZPCA9685_MODE2_OUTDRV        (byte)0x04
#define ZPCA9685_MODE2_OUTNE_HIZ        (byte)0x02


// Set to true to print some debug messages, or false to disable them.
//#define ENABLE_DEBUG_OUTPUT

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
  if (from == to) {
    if( value>=(uint32_t)(1<<to))
      value=(1<<to)-1;
    return value;
  }
  if (from > to) {
    return value >> (from-to);
  }
  return value << (to-from);
}
bool ZPCA9685::check()
  {
  return ((_i2caddr&(~0x3F))==0x40);}
void ZPCA9685::setHardAddress(uint8_t A543210)
  {
    _i2caddr=0x40 || A543210&0x3F;
  }
   
/**************************************************************************/
/*! 
    @brief  Instantiates a new ZPCA9685 PWM driver chip with the I2C address on the Wire interface. On Due we use Wire1 since its the interface on the 'default' I2C pins.
    @param  addr The 7-bit I2C address to locate this chip, default is 0x40
*/
/**************************************************************************/
ZPCA9685::ZPCA9685() : PinExtender() 
 {
  _writeResolution = 8;
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void ZPCA9685::begin(TwoWire *i2c,uint8_t addr)
  {
  _i2c = i2c;
  if ((addr & ZPCA9685_ADDRESS_MASK)!=ZPCA9685_ADDRESS) {
		while(1);
	}
  _i2caddr = addr;
  
   _i2c->begin();
  reset();
  // set a default frequency
  setPWMFreq(1526);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void ZPCA9685::begin(uint8_t addr)
  {
    begin(&Wire, addr);
}
/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware
*/
/**************************************************************************/

  void ZPCA9685::begin(void)
  {
	  begin( 0x40);
  }
  



/**************************************************************************/
/*! 
    @brief  Sends a reset command to the ZPCA9685 chip over I2C
*/
/**************************************************************************/
void ZPCA9685::reset(void) {
  SWRST ();// bus reset
  write8(ZPCA9685_MODE1, 0x80);
  delay(10);
   write8(ZPCA9685_MODE1_REG, ZPCA9685_MODE_RESTART | ZPCA9685_MODE_AUTOINC);
}
void ZPCA9685::SWRST (void) {
  //The SWRST Call function is defined as the following:
  //1.  A START command is sent by the I2C-bus master
  _i2c->beginTransmission(0);
  //2.  The reserved SWRST I2C-bus address ‘0000 000’ with the R/W bit set to ‘0’ (write) is sent by the I2C-bus master
  _i2c->write(0x06);
  /*
  The PCA9685 device(s) acknowledge(s) after seeing the General Call address 
‘0000 0000’ (00h) only. If the R/W
 bit is set to ‘1’ (read), no
 acknowledge is returned to 
the I2C-bus master.
4.  Once the General Call address has been sent and acknowledged, the master sends 
1 byte with 1 specific value (SWRST data byte 1):
a.  Byte 1 = 06h: the PCA9685 acknowledges th
is value only. If byte 1 is not equal to 
06h, the PCA9685 does not acknowledge it.
If more than 1 byte of data is sent, the PCA9685 does not acknowledge any more.
  
  5.Once the correct byte (SWRST data byte 1) has been sent and correctly 
acknowledged, the master sends a STOP command to end the SWRST Call: the 
PCA9685 then resets to the default value (power-up value) and is ready to be 
addressed again within the specified bus free time (t
BUF
)
*/
  _i2c->endTransmission();
  delay(10);
}
/**************************************************************************/
/*! 
    @brief  Sets the PWM frequency for the entire chip, up to ~1.6 KHz
    @param  freq Floating point frequency that we will attempt to match
*/
/**************************************************************************/
void ZPCA9685::setPWMFreq(float freq) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Attempting to set freq ");
  Serial.println(freq);
#endif

  freq *= 0.9;  // Correct for overshoot in the frequency setting (see issue #11).
  float prescaleval = 25000000;
  prescaleval /= 4096;
  prescaleval /= freq;
  prescaleval -= 1;

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Estimated pre-scale: "); Serial.println(prescaleval);
#endif

  uint8_t prescale = floor(prescaleval + 0.5);
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Final pre-scale: "); Serial.println(prescale);
#endif
  
  uint8_t oldmode = read8(ZPCA9685_MODE1);
  uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
  write8(ZPCA9685_MODE1, newmode); // go to sleep
  write8(ZPCA9685_PRESCALE, prescale); // set the prescaler
  write8(ZPCA9685_MODE1, oldmode);
  delay(5);
  write8(ZPCA9685_MODE1, oldmode | 0xa0);  //  This sets the MODE1 register to turn on auto increment.

#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Mode now 0x"); Serial.println(read8(ZPCA9685_MODE1), HEX);
#endif
}


bool ZPCA9685::acceptlocal(uint32_t p)
{
  return (p>>16==_i2caddr);
}

     
/** dummy function
*/
uint8_t ZPCA9685::digitalRead(uint32_t ulPin)
{
	if(acceptlocal( ulPin))
	{
		return 0;
	}
	else if (_next)
		return _next->digitalRead( ulPin);	
return 0;	
}
/**
 * Sets the pin mode to either INPUT or OUTPUT but for all, and input doesn't exist
 */
void ZPCA9685::pinMode(uint32_t ulPin, uint8_t mode) {
	
	if(acceptlocal( ulPin))
	{
	
	
	//uint8_t channel=pin2channel( ulPin);
	uint8_t tmp=read8( ZPCA9685_MODE2);
	tmp&=~7;
	#ifdef OUTPUT_LOW
		if (mode==OUTPUT_LOW)
		{}
			else
	#endif
		tmp|=ZPCA9685_MODE2_OUTDRV;	

if (
#ifdef INPUT_PULLDOWN
(mode==INPUT_PULLDOWN)||
#endif
(mode==INPUT_PULLUP) || (mode==INPUT))
	{tmp|=ZPCA9685_MODE2_OUTDRV;	}
		else
	tmp|=0;		

	write8( ZPCA9685_MODE2,tmp);
	}
	else if (_next)
		return _next->pinMode( ulPin,mode);
	
}
void ZPCA9685::analogWriteResolution(int res)
{
  _writeResolution = res;
  if (_next)
		return _next->analogWriteResolution( res);		
}
 void ZPCA9685::analogWrite( uint32_t ulPin, uint32_t ulValue ) 
 {
	 if(acceptlocal( ulPin))
	{
		
	 uint8_t channel=pin2channel(ulPin);
	  ulValue = mapResolution(ulValue, _writeResolution, 12);
	  if (( (channel > 15))&& (channel!=(uint8_t)-1)) return;//(channel < 0) ||
	 setPin( channel,  ulValue, false );
	}
	else if (_next)
		return _next->analogWrite( ulPin,ulValue);		
	
 }

 uint8_t ZPCA9685::pin2channel(uint32_t pin)
 {
	 if((pin>>16)==_i2caddr)
	 return pin &0xff;
 	 if((pin<16))
	 return pin &0xf;
 
 return NO_CHANNEL;
 }
 uint32_t ZPCA9685::analogRead( uint32_t pin )
{ 
  return 0;
}
void ZPCA9685::digitalWrite(uint32_t ulPin, uint8_t d)
{
	if(acceptlocal( ulPin))
	{
          uint8_t channel=pin2channel( ulPin);
          if(d==HIGH)
          {   if (( channel > 15)&& (channel!=(uint8_t)-1)) return;//channel < 0 ||
  setPWM( channel,  ZPCA9685_PWM_FULL,  0);
          }
          if (d==LOW)
          {
                      if (( channel > 15)&& (channel!=(uint8_t)-1)) return;//channel < 0 ||
  setPWM( channel, 0, ZPCA9685_PWM_FULL);

          }
	}
	else if (_next)
		return _next->digitalWrite( ulPin,d);	
}
/**************************************************************************/
/*! 
    @brief  Sets the PWM output of one of the ZPCA9685 pins
    @param  num One of the PWM output pins, from 0 to 15
    @param  on At what point in the 4096-part cycle to turn the PWM output ON
    @param  off At what point in the 4096-part cycle to turn the PWM output OFF
*/
/**************************************************************************/
void ZPCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {
#ifdef ENABLE_DEBUG_OUTPUT
  Serial.print("Setting PWM "); Serial.print(num); Serial.print(": "); Serial.print(on); Serial.print("->"); Serial.println(off);
#endif
byte regAddress;
  _i2c->beginTransmission(_i2caddr);
   if (num != 0xff)
        regAddress = LED0_ON_L + (num * 0x04);
    else
        regAddress = ZPCA9685_ALLLED_REG;
  _i2c->write(regAddress);
  _i2c->write(on);
  _i2c->write(on>>8);
  _i2c->write(off);
  _i2c->write(off>>8);
  _i2c->endTransmission();
}

/**************************************************************************/
/*! 
    @brief  Helper to set pin PWM output. Sets pin without having to deal with on/off tick placement and properly handles a zero value as completely off and 4095 as completely on.  Optional invert parameter supports inverting the pulse for sinking to ground.
    @param  num One of the PWM output pins, from 0 to 15
    @param  val The number of ticks out of 4096 to be active, should be a value from 0 to 4095 inclusive.
    @param  invert If true, inverts the output, defaults to 'false'
*/
/**************************************************************************/
void ZPCA9685::setPin(uint8_t num, uint16_t val, bool invert)
{
  // Clamp value between 0 and 4095 inclusive.
  val = min(val, (uint16_t)4095);
  if (invert) {
    if (val == 0) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 4095) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, 4095-val);
    }
  }
  else {
    if (val == 4095) {
      // Special value for signal fully on.
      setPWM(num, 4096, 0);
    }
    else if (val == 0) {
      // Special value for signal fully off.
      setPWM(num, 0, 4096);
    }
    else {
      setPWM(num, 0, val);
    }
  }
}

/*******************************************************************************************/

uint8_t ZPCA9685::read8(uint8_t addr) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->endTransmission();

  _i2c->requestFrom((uint8_t)_i2caddr, (uint8_t)1);
  return _i2c->read();
}

void ZPCA9685::write8(uint8_t addr, uint8_t d) {
  _i2c->beginTransmission(_i2caddr);
  _i2c->write(addr);
  _i2c->write(d);
  _i2c->endTransmission();
}


/*************************************************************************************/
 
#define SEROFREQ 50 //Hz
#define usToTicks(_us)    (((_us) *4096L)/(1000000L/SEROFREQ))                 // converts microseconds to tick
#define ticksToUs(_ticks) (  ((_ticks) *(1000000L/SEROFREQ))/4096L)   // converts from ticks back to microseconds
#define TRIM_DURATION  0                                   // compensation ticks to trim adjust for digitalWrite delays
#define MIN_PULSE_WIDTH       554     // the shortest pulse sent to a servo  
#define MAX_PULSE_WIDTH      2500     // the longest pulse sent to a servo 

uint8_t ZPCA9685::attach(int pin)
{
  return attach(pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}
/** attach a pin for servo motor
min : minimal us pulse, max max us pulse;
*/
  uint8_t ZPCA9685::attach(int ulPin, int min, int max) // as above but also sets min and max values for writes. 
  {  	
  if (!(acceptlocal( ulPin)|| ulPin<16))
	  return -1;
  uint8_t channel=pin2channel(ulPin);
   pinMode(channel, OUTPUT);
  setPWMFreq(SEROFREQ);  // freq must be setup
  //servos|=1<<channel;
  servo_min[channel]=min;
  servo_max[channel]=max;
	//   nh->subscribe(*subscriber);
	   return ulPin;

  }  
  void ZPCA9685::detach(int pin)
  {
	   uint8_t channel=pin2channel(pin);
	 pinMode(pin, INPUT_PULLUP);
	// servos&=~(1<<(pin&0xF));
	servo_max[channel] =0;
  //    subscriber->shutdown();
  }  
  
  void ZPCA9685::write(int pin,int value)             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  {
  uint8_t channel=pin2channel(pin);
	   // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
  if (value < MIN_PULSE_WIDTH)
  {
    if (value < 0)
      value = 0;
    else if (value > 180)
      value = 180;

    value = map(value, 0, 180, (servo_min[channel] ), ( servo_max[channel] ));
  }
  writeMicroseconds(pin,value);
  
  }  
  
  void ZPCA9685::writeMicroseconds(int ulPin,int value) // Write pulse width in microseconds 
  {
	  if (!(acceptlocal( ulPin)|| ulPin<16))
	  return;
  
  // calculate and store the values for the given channel
  uint8_t channel=pin2channel(ulPin);
  
  {
    if (value < (servo_min[channel] ))          // ensure pulse width is valid
      value = (servo_min[channel] );
    else if (value > ( servo_max[channel] ))
      value = ( servo_max[channel] );

    value = value - TRIM_DURATION;
    value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
    //servos[channel].ticks = value;
	setPin( channel, value, false);
	 
  }
  
  }  
  
  int ZPCA9685::read(int ulPin)                        // returns current pulse width as an angle between 0 and 180 degrees
  {
	  if (!(acceptlocal( ulPin)|| ulPin<16))
	  return -1;
  uint8_t channel=pin2channel(ulPin);
	 return map(readMicroseconds(ulPin)+1, (servo_min[channel] ), ( servo_max[channel] ), 0, 180);
  }  
  

  int ZPCA9685::readMicroseconds(int ulPin)            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  {
	  unsigned int pulsewidth;
  if (attached(ulPin))
    pulsewidth = ticksToUs(0/*readpwm servos[this->servoIndex].ticks*/)  + TRIM_DURATION;
  else
    pulsewidth  = 0;

  return pulsewidth;
  
  }  
   // return true if this servo is attached, otherwise false 
  bool ZPCA9685::attached(int ulPin)
  {
	  uint8_t channel=pin2channel(ulPin);
	 // return (servos&(1<<(channel)))!=0;  
	 
	// servos&=~(1<<(pin&0xF));
	return ( servo_max[channel] )==0;
  }

/*

ZPCA9685_ServoEvaluator pwmServo1;
void ZPCA9685::ServoForAngle(uint32_t ulPin, float  angle )
{
   setPWMFreq(50);  // 50Hz provides 20ms standard servo phase length
   setPin(ulPin, pwmServo1.pwmForAngle(angle));
}

ZPCA9685_ServoEvaluator::ZPCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t p90PWMAmount) {
    n90PWMAmount = constrain(n90PWMAmount, 0, ZPCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, n90PWMAmount, ZPCA9685_PWM_FULL);

    _coeff = new float[2];
    _isCSpline = false;

    _coeff[0] = n90PWMAmount;
    _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f;
}

ZPCA9685_ServoEvaluator::ZPCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount) {
    n90PWMAmount = constrain(n90PWMAmount, 0, ZPCA9685_PWM_FULL);
    zeroPWMAmount = constrain(zeroPWMAmount, n90PWMAmount, ZPCA9685_PWM_FULL);
    p90PWMAmount = constrain(p90PWMAmount, zeroPWMAmount, ZPCA9685_PWM_FULL);

    if (p90PWMAmount - zeroPWMAmount != zeroPWMAmount - n90PWMAmount) {
        _coeff = new float[8];
        _isCSpline = true;

        // Cubic spline code adapted from: https://shiftedbits.org/2011/01/30/cubic-spline-interpolation/
        //  "THE BEER-WARE LICENSE" (Revision 42): Devin Lane wrote this [part]. As long as you retain
        // this notice you can do whatever you want with this stuff. If we meet some day, and you
       // think this stuff is worth it, you can buy me a beer in return. 

        float x[3] = { 0, 90, 180 };
        float y[3] = { (float)n90PWMAmount, (float)zeroPWMAmount, (float)p90PWMAmount };
        float c[3], b[2], d[2], h[2], l[1], u[2], a[1], z[2]; // n = 3

        h[0] = x[1] - x[0];
        u[0] = z[0] = 0;
        c[2] = 0;

        for (int i = 1; i < 2; ++i) {
            h[i] = x[i + 1] - x[i];
            l[i - 1] = (2 * (x[i + 1] - x[i - 1])) - h[i - 1] * u[i - 1];
            u[i] = h[i] / l[i - 1];
            a[i - 1] = (3 / h[i]) * (y[i + 1] - y[i]) - (3 / h[i - 1]) * (y[i] - y[i - 1]);
            z[i] = (a[i - 1] - h[i - 1] * z[i - 1]) / l[i - 1];
        }

        for (int i = 1; i >= 0; --i) {
            c[i] = z[i] - u[i] * c[i + 1];
            b[i] = (y[i + 1] - y[i]) / h[i] - (h[i] * (c[i + 1] + 2 * c[i])) / 3;
            d[i] = (c[i + 1] - c[i]) / (3 * h[i]);

            _coeff[4 * i + 0] = y[i]; // a
            _coeff[4 * i + 1] = b[i]; // b
            _coeff[4 * i + 2] = c[i]; // c
            _coeff[4 * i + 3] = d[i]; // d
        }
    }
    else {
        _coeff = new float[2];
        _isCSpline = false;

        _coeff[0] = n90PWMAmount;
        _coeff[1] = (p90PWMAmount - n90PWMAmount) / 180.0f;
    }
}

ZPCA9685_ServoEvaluator::~ZPCA9685_ServoEvaluator() {
    if (_coeff) delete[] _coeff;
}

uint16_t ZPCA9685_ServoEvaluator::pwmForAngle(float angle) {
    float retVal;
    angle = constrain(angle + 90, 0, 180);

    if (!_isCSpline) {
        retVal = _coeff[0] + (_coeff[1] * angle);
    }
    else {
        if (angle <= 90) {
            retVal = _coeff[0] + (_coeff[1] * angle) + (_coeff[2] * angle * angle) + (_coeff[3] * angle * angle * angle);
        }
        else {
            angle -= 90;
            retVal = _coeff[4] + (_coeff[5] * angle) + (_coeff[6] * angle * angle) + (_coeff[7] * angle * angle * angle);
        }
    }
    
    return (uint16_t)constrain((int)roundf(retVal), 0, ZPCA9685_PWM_FULL);
};
*/


