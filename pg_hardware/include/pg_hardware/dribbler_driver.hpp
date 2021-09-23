#ifndef PG_DRIBBLER
#define PG_DRIBBLER

#ifdef __arm__
#include <wiringPi.h>
#endif

#include <cstdint>

namespace pg_ns {

#ifdef __arm__

class DribblerDriver {
	private:
	int8_t pin_a_;
	int8_t pin_b_;

	public:
	DribblerDriver();

	void init();
	void dribble();
	void stop();
};

#else

class DribblerDriver {
	private:
	bool cmd_dribble_;

	public:
	DribblerDriver();

	void setDribble();
	void stop();
	bool getCommandDribble();
};

}

#endif

#endif
