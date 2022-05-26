#ifndef PG_PROXI
#define PG_PROXI

#ifdef __aarch64__
extern "C" {
#include <wiringPi.h>
}
#endif

#include <cstdint>

namespace pg_ns {

#ifdef __aarch64__
class Proxi {
	private:
		uint8_t pin_;
		bool ball_in_range_;

	public:
		Proxi();
		void init(uint8_t pin);
		bool ballInRange();
}
#else

#endif

}

#endif
