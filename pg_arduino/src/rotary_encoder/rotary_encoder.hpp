// Copyright (c) 2021 EPROM PNUP
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom
// the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// Author: Wahyu Mahardika

#ifndef PG_ENCODER
#define PG_ENCODER

#include <Arduino.h>

namespace pg_ns {

class RotaryEncoder {
	private:
	byte en_c1_;
	byte en_c2_;
	volatile long pulse_count_ = 0;
	bool direction_cw_ = true;
	const static int16_t ENCODER_MIN = -32768;
	const static int16_t ENCODER_MAX = 32767;

	public:
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

