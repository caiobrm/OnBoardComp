#include "lib/libvector.h"

using namespace std;


int main()
{
	//rc_set_state(RUNNING);
	running = true;

	init_values_kf();
	init_sensors();

	while(running){


		rc_usleep(100000);
		fflush(stdout);
	}

	return 0;
}
