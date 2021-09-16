#ifndef PG_DRIBBLER
#define PG_DRIBBLER

#include <Arduino.h>

namespace pg_ns {

class Dribbler {
	private:
	byte dir_pin_1_;
	byte dir_pin_2_;

	public:
	Dribbler(byte dir_pin_1, byte dir_pin_2);

	void init();
	void dribble();
	void stop();
};

}

#endif
