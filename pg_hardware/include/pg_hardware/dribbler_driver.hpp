#ifndef PG_DRIBBLER
#define PG_DRIBBLER

#include <cstdint>

namespace pg_ns {

class DribblerDriver {
	private:
	bool cmd_dribble_;

	public:
	DribblerDriver();

	void dribble();
	void stop();
	bool getCommandDribble();
};

}

#endif
