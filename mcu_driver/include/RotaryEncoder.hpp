#ifndef PG_ENCODER
#define PG_ENCODER

#include <Arduino.h>

namespace pg_ns {

/* ROTARY ENCODER CLASS */
class RotaryEncoder {

	private:
	byte _en_c1;
	byte _en_c2;
	volatile long _pulse_count = 0;
	bool _direction_cw = true;
	const static int16_t _ENCODER_MIN = -32768;
	const static int16_t _ENCODER_MAX = 32767;

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
