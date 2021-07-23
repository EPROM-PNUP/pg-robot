#ifndef PG_ENCODER
#define PG_ENCODER

#include <Arduino.h>

namespace pg_ns {

/* ROTARY ENCODER CLASS */
class RotaryEncoder {

	private:
	byte en_c1_;
	byte en_c2_;
	volatile long pulse_count_ = 0;
	bool direction_cw_ = true;
	const static int16_t ENCODER_MIN = -32768;
	const static int16_t ENCODER_MAX = 32767;

	public:
	const static int16_t INTERVAL = 30;

	RotaryEncoder(const byte &en_c1, const byte &en_c2);
	void init();
	byte getC1Pin();

	friend void pulseInterruptA();
	friend void pulseInterruptB();
	friend void pulseInterruptC();
};

void pulseInterruptA();
void pulseInterruptB();
void pulseInterruptC();

}

#endif
