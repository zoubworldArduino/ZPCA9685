/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#ifndef _PCA9685_H
#define _PCA9685_H


#include "PinExtender.h"
   


#define  PCA9685_LED15 15
#define  PCA9685_LED14 14
#define  PCA9685_LED13 13
#define  PCA9685_LED12 12
#define  PCA9685_LED11 11
#define  PCA9685_LED10 10
#define  PCA9685_LED9 9
#define  PCA9685_LED8 8

#define  PCA9685_LED7 7
#define  PCA9685_LED6 6
#define  PCA9685_LED5 5
#define  PCA9685_LED4 4
#define  PCA9685_LED3 3
#define  PCA9685_LED2 2
#define  PCA9685_LED1 1
#define  PCA9685_LED0 0


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with ZPCA9685 PWM chip
*/
/**************************************************************************/
class ZPCA9685 : public PinExtender  {
 public:
 ZPCA9685();

   /** @name the Arduino like API
*/
  void pinMode(uint32_t p, uint8_t d);
  void digitalWrite(uint32_t p, uint8_t d);
  uint8_t digitalRead(uint32_t p);
  /*
 * \brief Writes an analog value (PWM wave) to a pin.
 *
 * \param ulPin
 * \param ulValue
 */
 void analogWrite( uint32_t ulPin, uint32_t ulValue ) ;
/*
 * \brief Set the resolution of analogWrite parameters. Default is 8 bits (range from 0 to 255).
 *
 * \param res
 */
  void analogWriteResolution(int res);
  
  /*
  void ServoForAngle(uint32_t ulPin, float  angle );
*/  uint32_t analogRead( uint32_t pin 
  );
//@}
   /** @name API for hardware management
*/
//@{
	
	 /** initialise the component,
  The Wire interface must be initialize before. see wire.begin()
  */
	  void begin(TwoWire *MyWire//!< the Wire interface like &Wire for board that handle several one.
	  ,uint8_t addr//!< the I2C address of PCA9685
	  );
	  	 /** initialise the component,
  The Wire interface must be initialize before. see wire.begin()
  */
  void begin(uint8_t addr//!< the I2C address of PCA9685
  );
  	 /** initialise the component,
  The Wire interface must be initialize before. see wire.begin()
  default Wire and addresses will be used : #PCA9685_ADDR_BASE.
  */
  void begin(void);
  /** Reset the board like a power up.
  
  
  */
  void SWRST (void);
   /**  check the board
  @deprecated
  */
bool check();
/** test the hardware
@return true if all is ok.
*/
bool test();
  /** setup the I2C device address on wire interface
  */
void setHardAddress(uint8_t A543210//!< the I2C adress bit 5...0 as define on pin of PCA9685
);
  /** reset the component
  */
  void reset(void);
   /** set up the frequency of the PWM
 note max is about 1500 Hz

*/
  void setPWMFreq(float freq//!< the frequency in Hz
  );
  /** setup the PWM wave form  :
  - "t=0                                         4096"
  - "   _______________------------------________."
  - "                 ON                OFF          "
  .
  

  */
  void setPWM(
  uint8_t num //!< the channel number
  , uint16_t on //!< the on start time  0..4095, if 4096 it will be always on
  , uint16_t off//!< the off start time  0..4095, if 4096 it will be always off(have the priority on on=4096)
  );
  /** setup a PWM
  */
  void setPin(uint8_t num//!< the channel number
  , uint16_t val,//!< the value 0..4095
  bool invert=false//!< the inversion of polarity of the waveform
  );


//  protected:
       bool acceptlocal(uint32_t p);
  //     friend Zmotor2;
  
  
//@}
   /** @name API for servo motor
*/
//@{
  

uint8_t attach(int pin//!< the pin number (specific to this instance)
);
/** attach a pin for servo motor
*/
  uint8_t attach(int pin//!< the pin number (specific to this instance)
  ,  int min //!< the min angle, default 0
  ,   int max//!<the max angle, default 180
  ) ;
  void detach(int pin//!< the pin number (specific to this instance)
  );
  
  void write(int pin//!< the pin number (specific to this instance)
  ,int value // the angle value in degre
  );
  void writeMicroseconds(int ulPin//!< the pin number (specific to this instance)
  ,int value // the value in microsecond
  );
  
  int read(int ulPin//!< the pin number (specific to this instance)
  ); 
  
  /**
  */
  int readMicroseconds(int ulPin//!< the pin number (specific to this instance)
  )  ;
   /** return true if this servo is attached, otherwise false 
   */
  bool attached(int ulPin//!< the pin number (specific to this instance)
  );
//@}
 private: 
 
 
//uint16_t servos=0;
uint16_t servo_min[16];
uint16_t servo_max[16];




  int _writeResolution;
  //uint8_t _i2caddr;
   uint8_t pin2channel(uint32_t pin);
  //TwoWire *_i2c;

  uint8_t read8(uint8_t addr);
  void write8(uint8_t addr, uint8_t d);
};

/*
// Class to assist with calculating Servo PWM values from angle values
class ZPCA9685_ServoEvaluator {
public:
    // Uses a linear interpolation method to quickly compute PWM output value. Uses
    // default values of 2.5% and 12.5% of phase length for -90/+90.
    ZPCA9685_ServoEvaluator(uint16_t n90PWMAmount = 102, uint16_t p90PWMAmount = 512);

    // Uses a cubic spline to interpolate due to an offsetted zero angle that isn't
    // exactly between -90/+90. This takes more time to compute, but gives a more
    // accurate PWM output value along the entire range.
    ZPCA9685_ServoEvaluator(uint16_t n90PWMAmount, uint16_t zeroPWMAmount, uint16_t p90PWMAmount);

    ~ZPCA9685_ServoEvaluator();

    // Returns the PWM value to use given the angle (-90 to +90)
    uint16_t pwmForAngle(float angle);

private:
  
  //PinExtender * _next;
    float *_coeff;      // a,b,c,d coefficient values
    bool _isCSpline;    // Cubic spline tracking, for _coeff length
};
*/

#endif
