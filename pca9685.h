//基于wiringPi的I2C库 (参考Arduino上的驱动Adafruit_PWMServoDriver.cpp)
//H.Z. Sang @Shanghai    20171101
#ifndef __PCA9685_H
#define __PCA9685_H
#include <stdint.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#define min(x,y) ((x)>(y)?(y):(x))
#define max(x,y) ((x)>(y)?(x):(y))


#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD

class PCA9685
{
	public:
		PCA9685(uint8_t addr = 0x40);
		virtual ~PCA9685();
		virtual void setPWMFreq(double freq);
		virtual void setPWM(uint8_t num, uint16_t on, uint16_t off);
		virtual void setPin(uint8_t num, uint16_t val, bool invert=false); //??
		virtual void reset();//

		virtual void setPulse(uint8_t num, double Pulse);
		virtual void rot(uint8_t num,uint8_t angle);
	
	private:
		uint8_t devID;
		int fd;
		double _freq;
};

PCA9685::PCA9685(uint8_t addr)
{
	devID=addr;
	fd=wiringPiI2CSetup(devID);
};
PCA9685::~PCA9685()
{
};

void PCA9685::reset()
{
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, 0x0);
};

void PCA9685::setPWMFreq(double freq)
{
	_freq=freq;
	freq *= 0.915;  // Correct for overshoot in the frequency setting (see issue #11).
	double osc_clock = 25000000.0;//inner clock:25MHZ
	uint8_t prescale = (int)(  (osc_clock/freq)/4096.0 - 1  +  0.5  ); //4096=2^12 : 12-bit  //floor


	uint8_t oldmode = wiringPiI2CReadReg8(fd, PCA9685_MODE1);
	uint8_t newmode = (oldmode&0x7F) | 0x10; // sleep
	
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, newmode);//go to sleep
	wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);//set the prescale
	oldmode &= 0xef;	//清除sleep位   ??
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, oldmode);
	delay(5);
	wiringPiI2CWriteReg8(fd, PCA9685_MODE1, oldmode | 0xa1);//This sets the MODE1 register to turn on auto increment.
};

void PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off)
{
	wiringPiI2CWriteReg16(fd, LED0_ON_L+4*num, on);
	wiringPiI2CWriteReg16(fd, LED0_OFF_L+4*num, off);
};


void PCA9685::setPulse(uint8_t num, double pulse)//这里的脉宽pulse单位是ms
{
	double pulselength = 1000;   // 1,000 ms per second
	pulselength /= _freq;   //50 Hz 60 Hz
	pulselength /= 4096;  // 12 bits of resolution, 2^12=4096
	pulse /= pulselength;

	uint16_t on=0; //每次周期一开始就输出高电平
	uint16_t off=pulse*1.0067114; //1.0067114校准?

	setPWM(num, on, off);
};

void PCA9685::rot(uint8_t num,uint8_t angle)
{
	double pulse=0.5+angle/90.0;//pluse单位ms, 舵机脉宽500us~2500us对应角度0~180°
	setPulse(num,pulse);
};


// Sets pin without having to deal with on/off tick placement and properly handles
// a zero value as completely off.  Optional invert parameter supports inverting
// the pulse for sinking to ground.  Val should be a value from 0 to 4095 inclusive.
void PCA9685::setPin(uint8_t num, uint16_t val, bool invert)
{
	// Clamp value between 0 and 4095 inclusive.
	val = min(val, 4095);
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
};

#endif
