#ifndef PG_DRIBBLER
#define PG_DRIBBLER

#include <Arduino.h>

namespace pg_ns {

class Dribbler {
	private:
	byte dir_pin_1_;
	byte dir_pin_2_;

	int16_t duty_cycle_;

	public:
	Dribbler(byte dir_pin_1, byte dir_pin_2);

	void init();
	int16_t clip(int16_t value, int16_t minimum, int16_t maximum); 
	void dribble(int16_t pwm);
};

}

#endif
