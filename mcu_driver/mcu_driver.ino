/* INCLUDE HEADERS */
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include "src/motor/motor.hpp"
#include "src/rotary_encoder/rotary_encoder.hpp"
#include "src/cmps12/cmps12.hpp"



/* OBJECTS INITIALIZATION */
// Motors A, B, C
pg_ns::Motor motor_a(4, 5);
pg_ns::Motor motor_b(8, 9);
pg_ns::Motor motor_c(6, 7);
// Encoders A, B, C
pg_ns::RotaryEncoder re_a(2, 3);
pg_ns::RotaryEncoder re_b(18, 19);
pg_ns::RotaryEncoder re_c(20, 21);
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
	attachInterrupt(digitalPinToInterrupt(re_a.getC1Pin()), pg_ns::pulseInterruptA, RISING);
	attachInterrupt(digitalPinToInterrupt(re_b.getC1Pin()), pg_ns::pulseInterruptB, RISING);
	attachInterrupt(digitalPinToInterrupt(re_c.getC1Pin()), pg_ns::pulseInterruptC, RISING);

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
	if(current_millis - encoders_millis_track > pg_ns::RotaryEncoder::INTERVAL) {
		encoders_millis_track = current_millis;

		encoders_pulse_count_pub.publish(&encoders_pulse_count);
	}

	// Publish imu sensor raw data every 30 ms
	if(current_millis - imu_millis_track > pg_ns::CMPS12::INTERVAL) {
		imu_millis_track = current_millis;

		imu_data_raw = imu.getRawData();

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



/* ENCODERS FUNCTION INTERRUPT */
// Wheel A interrupt
void pg_ns::pulseInterruptA() {
	if(digitalRead(re_a.en_c2_) == LOW) {
		re_a.direction_cw_ = true;
	}
	else {
		re_a.direction_cw_ = false;
	}

	if(re_a.direction_cw_) {
		if(re_a.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_a.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoders_pulse_count.data[0] = re_a.pulse_count_;
		}
		else {
			re_a.pulse_count_++;
			encoders_pulse_count.data[0] = re_a.pulse_count_;
		}
	}
	else {
		if(re_a.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_a.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoders_pulse_count.data[0] = re_a.pulse_count_;
		}
		else {
			re_a.pulse_count_--;
			encoders_pulse_count.data[0] = re_a.pulse_count_;
		}
	}
}

// Wheel B interrupt
void pg_ns::pulseInterruptB() {
	if(digitalRead(re_b.en_c2_) == LOW) {
		re_b.direction_cw_ = true;
	}
	else {
		re_b.direction_cw_ = false;
	}

	if(re_b.direction_cw_) {
		if(re_b.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_b.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoders_pulse_count.data[1] = re_b.pulse_count_;
		}
		else {
			re_b.pulse_count_++;
			encoders_pulse_count.data[1] = re_b.pulse_count_;
		}
	}
	else {
		if(re_b.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_b.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoders_pulse_count.data[1] = re_b.pulse_count_;
		}
		else {
			re_b.pulse_count_--;
			encoders_pulse_count.data[1] = re_b.pulse_count_;
		}
	}
}

// Wheel C interrupt
void pg_ns::pulseInterruptC() {
	if(digitalRead(re_c.en_c2_) == LOW) {
		re_c.direction_cw_ = true;
	}
	else {
		re_c.direction_cw_ = false;
	}

	if(re_c.direction_cw_) {
		if(re_c.pulse_count_ == RotaryEncoder::ENCODER_MAX) {
			re_c.pulse_count_ = RotaryEncoder::ENCODER_MIN;
			encoders_pulse_count.data[2] = re_c.pulse_count_;
		}
		else {
			re_c.pulse_count_++;
			encoders_pulse_count.data[2] = re_c.pulse_count_;
		}
	}
	else {
		if(re_c.pulse_count_ == RotaryEncoder::ENCODER_MIN) {
			re_c.pulse_count_ = RotaryEncoder::ENCODER_MAX;
			encoders_pulse_count.data[2] = re_c.pulse_count_;
		}
		else {
			re_c.pulse_count_--;
			encoders_pulse_count.data[2] = re_c.pulse_count_;
		}
	}
}


















