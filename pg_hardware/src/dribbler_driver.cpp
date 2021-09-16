#include "pg_hardware/dribbler_driver.hpp"

namespace pg_ns {

DribblerDriver::DribblerDriver() {
	cmd_dribble_ = false;
}

void DribblerDriver::dribble() {
	cmd_dribble_ = true;
}

void DribblerDriver::stop() {
	cmd_dribble_ = false;
}

bool DribblerDriver::getCommandDribble() {
	return cmd_dribble_;
}

}

