/* INCLUDE HEADERS */
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "cmps12/CMPS12.hpp"



/* ROTARY ENCODER CLASS DEFINITION */
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

	RotaryEncoder(const byte &en_c1, const byte &en_c2) {
		this->_en_c1 = en_c1;
		this->_en_c2 = en_c2;
	}

	void init() {
		pinMode(this->_en_c1, INPUT_PULLUP);
		pinMode(this->_en_c2, INPUT);
	}

	byte getC1Pin() {
		return this->_en_c1;
	}

	friend void _pulse_a();
	friend void _pulse_b();
	friend void _pulse_c();
};


/* MOTOR CLASS DEFINITION */
class Motor{
	
	private:
	byte _dir_pin_1;
	byte _dir_pin_2;
	int16_t _duty_cycle;

	public:

	Motor(const byte &dir_pin_1, const byte &dir_pin_2) {
		this->_dir_pin_1 = dir_pin_1;
		this->_dir_pin_2 = dir_pin_2;
	}

	void init() {
		pinMode(this->_dir_pin_1, OUTPUT);
		pinMode(this->_dir_pin_2, OUTPUT);
	}

	int16_t _clip(const int16_t &value, const int16_t &minimum, const int16_t &maximum) {
		if(value > maximum) {
			return maximum;
		}
		else if(value < minimum) {
			return minimum;
		}
		return value;
	}

	int16_t getDutyCycle() {
		return this->_duty_cycle;
	}

	void move(const int16_t &speed) {
		this->_duty_cycle = _clip(abs(speed), 0, 255);
		if(speed < 0) {
			analogWrite(this->_dir_pin_1, this->_duty_cycle);
			analogWrite(this->_dir_pin_2, 0);
		}
		else {
			analogWrite(this->_dir_pin_1, 0);
			analogWrite(this->_dir_pin_2, this->_duty_cycle);
		}
	}
};


/* OBJECTS INITIALIZATION */
// Motors A, B, C
Motor motor_a(4, 5);
Motor motor_b(8, 9);
Motor motor_c(6, 7);
// Encoders A, B, C
RotaryEncoder re_a(2, 3);
RotaryEncoder re_b(18, 19);
RotaryEncoder re_c(20, 21);
// IMU Sensor
pg_ns::CMPS12 imu;


/* CALLBACK FUNCTIONS */
void motorCallback(const std_msgs::Int16MultiArray &speed) {
	motor_a.move(speed.data[0]);
	motor_b.move(speed.data[1]);
	motor_c.move(speed.data[2]);
}


/* ROS NODE HANDLE & MSGS */
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Int16MultiArray> motor_speed_sub("motor_pwm", &motorCallback);

// Encoders Publisher & msg
std_msgs::Int16MultiArray encoders_pulse_count;
ros::Publisher encoders_pulse_count_pub("encoders_pulse_count", &encoders_pulse_count);
int16_t temp_3[3] = {0, 0, 0};

// IMU Publisher & msg
std_msgs::Int16MultiArray imu_data_raw_msg;
ros::Publisher imu_pub("imu_data_raw", &imu_data_raw_msg);
int16_t temp_9[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};


/* ENCODERS FUNCTION INTERRUPT */
// Wheel A interrupt
void _pulse_a() {
	if(digitalRead(re_a._en_c2) == LOW) {
		re_a._direction_cw = true;
	}
	else {
		re_a._direction_cw = false;
	}

	if(re_a._direction_cw) {
		if(re_a._pulse_count == RotaryEncoder::_ENCODER_MAX) {
			re_a._pulse_count = RotaryEncoder::_ENCODER_MIN;
			encoders_pulse_count.data[0] = re_a._pulse_count;
		}
		else {
			re_a._pulse_count++;
			encoders_pulse_count.data[0] = re_a._pulse_count;
		}
	}
	else {
		if(re_a._pulse_count == RotaryEncoder::_ENCODER_MIN) {
			re_a._pulse_count = RotaryEncoder::_ENCODER_MAX;
			encoders_pulse_count.data[0] = re_a._pulse_count;
		}
		else {
			re_a._pulse_count--;
			encoders_pulse_count.data[0] = re_a._pulse_count;
		}
	}
}
// Wheel B interrupt
void _pulse_b() {
	if(digitalRead(re_b._en_c2) == LOW) {
		re_b._direction_cw = true;
	}
	else {
		re_b._direction_cw = false;
	}

	if(re_b._direction_cw) {
		if(re_b._pulse_count == RotaryEncoder::_ENCODER_MAX) {
			re_b._pulse_count = RotaryEncoder::_ENCODER_MIN;
			encoders_pulse_count.data[1] = re_b._pulse_count;
		}
		else {
			re_b._pulse_count++;
			encoders_pulse_count.data[1] = re_b._pulse_count;
		}
	}
	else {
		if(re_b._pulse_count == RotaryEncoder::_ENCODER_MIN) {
			re_b._pulse_count = RotaryEncoder::_ENCODER_MAX;
			encoders_pulse_count.data[1] = re_b._pulse_count;
		}
		else {
			re_b._pulse_count--;
			encoders_pulse_count.data[1] = re_b._pulse_count;
		}
	}
}
// Wheel C interrupt
void _pulse_c() {
	if(digitalRead(re_c._en_c2) == LOW) {
		re_c._direction_cw = true;
	}
	else {
		re_c._direction_cw = false;
	}

	if(re_c._direction_cw) {
		if(re_c._pulse_count == RotaryEncoder::_ENCODER_MAX) {
			re_c._pulse_count = RotaryEncoder::_ENCODER_MIN;
			encoders_pulse_count.data[2] = re_c._pulse_count;
		}
		else {
			re_c._pulse_count++;
			encoders_pulse_count.data[2] = re_c._pulse_count;
		}
	}
	else {
		if(re_c._pulse_count == RotaryEncoder::_ENCODER_MIN) {
			re_c._pulse_count = RotaryEncoder::_ENCODER_MAX;
			encoders_pulse_count.data[2] = re_c._pulse_count;
		}
		else {
			re_c._pulse_count--;
			encoders_pulse_count.data[2] = re_c._pulse_count;
		}
	}
}


/* GLOBAL VARIABLES */
static long encoders_millis_track = 0;
static long imu_millis_track = 0;
static long current_millis = 0;

static pg_ns::ImuDataRaw imu_data_raw;


/* ARDUINO SETUP FUNCTION */
void setup() {

	// Initialize encoder objects
	re_a.init();
	re_b.init();
	re_c.init();

	// Attach interrupt on every encoder C1 pins
	attachInterrupt(digitalPinToInterrupt(re_a.getC1Pin()), _pulse_a, RISING);
	attachInterrupt(digitalPinToInterrupt(re_b.getC1Pin()), _pulse_b, RISING);
	attachInterrupt(digitalPinToInterrupt(re_c.getC1Pin()), _pulse_c, RISING);

	// Initialize dc motor objects
	motor_a.init();
	motor_b.init();
	motor_c.init();

	// Initialize imu sensor object
	imu.init();
	
	// Initialize ROS node and baudrate
	nh.getHardware()->setBaud(115200);
	nh.initNode();

	// Initialize subscribers 
	nh.subscribe(motor_speed_sub);

	// Initialize publishers
	nh.advertise(encoders_pulse_count_pub);
	nh.advertise(imu_pub);

	// Initialize msgs data length
	encoders_pulse_count.data_length = 3;
	encoders_pulse_count.data = temp_3;

	imu_data_raw_msg.data_length = 9;
	imu_data_raw_msg.data = temp_9;
}

/* ARDUINO LOOP FUNCTION */
void loop() {
	nh.spinOnce();

	current_millis = millis();
	
	// Publish encoders pulse counts every 30 ms
	if(current_millis - encoders_millis_track > RotaryEncoder::INTERVAL) {
		encoders_millis_track = current_millis;

		encoders_pulse_count_pub.publish(&encoders_pulse_count);
	}

	// Publish imu sensor raw data every 30 ms
	if(current_millis - imu_millis_track > pg_ns::CMPS12::INTERVAL) {
		imu_millis_track = current_millis;

		imu_data_raw_msg.data[0] = imu_data_raw.bearing_;
		imu_data_raw_msg.data[1] = imu_data_raw.pitch_;
		imu_data_raw_msg.data[2] = imu_data_raw.roll_;

		imu_data_raw_msg.data[3] = imu_data_raw.gyro_x_;
		imu_data_raw_msg.data[4] = imu_data_raw.gyro_y_;
		imu_data_raw_msg.data[5] = imu_data_raw.gyro_z_;

		imu_data_raw_msg.data[6] = imu_data_raw.accel_x_;
		imu_data_raw_msg.data[7] = imu_data_raw.accel_y_;
		imu_data_raw_msg.data[8] = imu_data_raw.accel_z_;

		imu_pub.publish(&imu_data_raw_msg);
	}
}


















