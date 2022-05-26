#ifndef PG_KICKER
#define PG_KICKER

#ifdef __aarch64__
extern "C" {
#include <wiringPi.h>
}
#endif

#include <cstdint>

namespace pg_ns {

#ifdef __aarch64__

class Kicker {
	private:
	int8_t charge_pin_;
	int8_t release_pin_;

	bool ready_;

	public:
	Kicker();

	void init(int8_t charge_pin, int8_t release_pin);
	void charge();
	void setReady();
	void release();
	bool isReady();
};

#else

#endif

}

#endif
	
