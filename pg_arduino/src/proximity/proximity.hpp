#ifndef PG_PROXIMITY
#define PG_PROXIMITY

#include <Arduino.h>

namespace pg_ns {

class Proximity {
	private:
	byte pin_;
	bool ball_in_range_;

	public:
	Proximity(byte pin);

	void init();
	bool ballIsInRange();
};

}

#endif

