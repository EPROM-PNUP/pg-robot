#ifndef PG_MOTOR
#define PG_MOTOR

#include <Arduino.h>

namespace pg_ns {

/* MOTOR CLASS DEFINITION */
class Motor{
	
	private:
	byte _dir_pin_1;
	byte _dir_pin_2;
	int16_t _duty_cycle;

	public:

	Motor(const byte &dir_pin_1, const byte &dir_pin_2);

	void init();
	int16_t _clip(const int16_t &value, const int16_t &minimum, const int16_t &maximum);
	int16_t getDutyCycle();
	void move(const int16_t &speed);
};


}

#endif
