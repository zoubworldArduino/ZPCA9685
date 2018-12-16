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
  void begin(TwoWire *MyWire,uint8_t addr);
  void begin(uint8_t addr);
  void begin(void);
  void SWRST (void);
bool check();
bool test();
  
void setHardAddress(uint8_t A543210);
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
*/
  
  void reset(void);
  void setPWMFreq(float freq);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void setPin(uint8_t num, uint16_t val, bool invert=false);
  uint32_t analogRead( uint32_t pin );

//  protected:
       bool acceptlocal(uint32_t p);
  //     friend Zmotor2;
  
  
  
  
  

uint8_t attach(int pin);
/** attach a pin for servo motor
min : minimal us pulse, max max us pulse;
*/
  uint8_t attach(int pin, int min, int max) ;
  void detach(int pin);
  
  void write(int pin,int value);
  void writeMicroseconds(int ulPin,int value);
  
  int read(int ulPin); 
  

  int readMicroseconds(int ulPin)  ;
   // return true if this servo is attached, otherwise false 
  bool attached(int ulPin);
  
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
