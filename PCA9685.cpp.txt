#include "PCA9685.h"

#include <wiringPi.h>
#include <wiringPiI2C.h>

// Registers/etc.
#define __MODE1             0x00
#define __MODE2             0x01
#define __SUBADR1           0x02
#define __SUBADR2           0x03
#define __SUBADR3           0x04
#define __PRESCALE          0xFE
#define __LED0_ON_L         0x06
#define __LED0_ON_H         0x07
#define __LED0_OFF_L        0x08
#define __LED0_OFF_H        0x09
#define __ALL_LED_ON_L      0xFA
#define __ALL_LED_ON_H      0xFB
#define __ALL_LED_OFF_L     0xFC
#define __ALL_LED_OFF_H     0xFD

// Bits
#define __RESTART           0x80
#define __SLEEP             0x10
#define __ALLCALL           0x01
#define __INVRT             0x10
#define __OUTDRV  			0x04 

#define MAX_SPEED_L       	230
#define MAX_SPEED       	255
#define MAX_SPEED_R       	230
#define NOR_SPEED       	70
#define MIN_SPEED       	90
#define SPEED				85

#define FR1_SPEED			40
#define BK1_SPEED			60


#define MOTOR_START_DELAY	5 


PCA9685::PCA9685()
{
    enAPin = 0;
	en1Pin = 1;
	en2Pin = 2;
	en3Pin = 3;
	en4Pin = 4;
	enBPin = 5;
    BuzzPin = 8;
	
	nSpeed = NOR_SPEED;
	frSpeed = FR1_SPEED;
	bkSpeed = BK1_SPEED;
    
    init();
}

PCA9685::~PCA9685()
{
    setPWM(en1Pin, 0, 0);
    setPWM(en2Pin, 0, 0);
    setPWM(en3Pin, 0, 0);
    setPWM(en4Pin, 0, 0);

}

void PCA9685::init()
{   
    unsigned char mode1;
    
    fd = wiringPiI2CSetup(0x60);
    
    //wiringPiI2CWrite(fd, 0x06); // SWRST
    setAllPWM(0, 0);
    wiringPiI2CWriteReg8(fd, __MODE2, __OUTDRV);
    wiringPiI2CWriteReg8(fd, __MODE1, __ALLCALL);
    delay(5);                                       // wait for oscillator
    
    mode1 = wiringPiI2CReadReg8(fd, __MODE1);
    mode1 = mode1 & ~__SLEEP;                 // wake up (reset sleep)
    wiringPiI2CWriteReg8(fd, __MODE1, mode1);
    delay(5);
    
    setPWMFreq(1000);
}

void PCA9685::setPWMFreq(int freq)
{
    unsigned char oldmode, newmode;
    float prescaleval;

    // Sets the PWM frequency
    prescaleval = 25000000.0;    // 25MHz
    prescaleval /= 4096.0;       // 12-bit
    prescaleval /= freq;
    prescaleval -= 0.5;

    oldmode = wiringPiI2CReadReg8(fd, __MODE1);
    newmode = (oldmode & 0x7F) | 0x10;            // sleep
    wiringPiI2CWriteReg8(fd, __MODE1, newmode);   // go to sleep
    wiringPiI2CWriteReg8(fd, __PRESCALE, (unsigned char)prescaleval);
    wiringPiI2CWriteReg8(fd, __MODE1, oldmode);
    delay(5);
    wiringPiI2CWriteReg8(fd, __MODE1, oldmode | 0x80);    
}

void PCA9685::setPWM(int channel, int on, int off)
{
    // Sets a single PWM channel
    wiringPiI2CWriteReg8(fd, __LED0_ON_L+4*channel, on & 0xFF);
    wiringPiI2CWriteReg8(fd, __LED0_ON_H+4*channel, on >> 8);
    wiringPiI2CWriteReg8(fd, __LED0_OFF_L+4*channel, off & 0xFF);
    wiringPiI2CWriteReg8(fd, __LED0_OFF_H+4*channel, off >> 8);
}

void PCA9685::setAllPWM(int on, int off)
{
    // Sets a all PWM channels
    wiringPiI2CWriteReg8(fd, __ALL_LED_ON_L, on & 0xFF);
    wiringPiI2CWriteReg8(fd, __ALL_LED_ON_H, on >> 8);
    wiringPiI2CWriteReg8(fd, __ALL_LED_OFF_L, off & 0xFF);
    wiringPiI2CWriteReg8(fd, __ALL_LED_OFF_H, off >> 8);
}

void PCA9685::setPin(int pin, int value)
{
    if(value == 0)
        setPWM(pin, 0, 4096);
    if(value == 1)
        setPWM(pin, 4096, 0);
}

void PCA9685::goForward(void)
{
	setPin(en1Pin, HIGH);
    setPin(en2Pin, LOW);

    setPin(en3Pin, HIGH);
    setPin(en4Pin, LOW);
	
	setSpeed(enAPin, MAX_SPEED);
    setSpeed(enBPin, MAX_SPEED);
    stop();
    delay(10);
    setSpeed(enAPin, frSpeed);
    setSpeed(enBPin, bkSpeed);  
}

void PCA9685::goBack(void)
{
    setPin(en1Pin, HIGH);
    setPin(en2Pin, LOW);

    setPin(en3Pin, HIGH);
    setPin(en4Pin, LOW);

	setSpeed(enAPin, FR1_SPEED);
    setSpeed(enBPin, BK1_SPEED);
    
	delay(40); // 10ms
    setSpeed(enAPin, FR1_SPEED);
    setSpeed(enBPin, BK1_SPEED);
}

void PCA9685::goLeft(void)
{
    setPin(en1Pin, HIGH);
    setPin(en2Pin, LOW);

    setPin(en3Pin, HIGH);
    setPin(en4Pin, LOW);

	setSpeed(enAPin, MAX_SPEED_L);//fr left and br right
    setSpeed(enBPin, MAX_SPEED_L);
	delay(MOTOR_START_DELAY); // 10ms
    setSpeed(enAPin, frSpeed);
    setSpeed(enBPin, bkSpeed);
}

void PCA9685::goRight(void)
{
    setPin(en1Pin, HIGH);
    setPin(en2Pin, LOW);

    setPin(en3Pin, HIGH);
    setPin(en4Pin, LOW);

	setSpeed(enAPin, MAX_SPEED_L);
    setSpeed(enBPin, MAX_SPEED_R);
	delay(MOTOR_START_DELAY); // 10ms
    setSpeed(enAPin, frSpeed);
    setSpeed(enBPin, bkSpeed);
}

void PCA9685::stop(void)
{
    setSpeed(enAPin, 0);
    setSpeed(enBPin, 0);    
    
}

void PCA9685::setSpeed(int pin, int speed)
{
	if (speed < 0)
		speed = 0;
	if (speed > 255)
		speed = 255;
	setPWM(pin, 0, speed*16);    
}

void PCA9685::setNormalSpeed(int speed)
{
	nSpeed = speed;
}
void PCA9685::setModSpeed(int fr, int bk)
{
	frSpeed = fr;
	bkSpeed = bk;
}

void PCA9685::onBuzz(void)
{
    setPWM(BuzzPin, 0, 2048);  
}

void PCA9685::offBuzz(void)
{
    setPin(BuzzPin, 0);
}
