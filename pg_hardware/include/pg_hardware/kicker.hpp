#ifndef PG_KICKER
#define PG_KICKER

#include <cstdint>

namespace pg_ns {

#ifdef __arm__

class Kicker {
	private:
	int8_t charge_pin_;
	int8_t release_pin_;

	bool ready_;

	public:
	Kicker(int8_t charge_pin, int8_t release_pin);

	void init();
	void charge();
	void release();
	bool isReady();
};

#else

#endif

}

#endif
	
