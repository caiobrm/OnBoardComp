#include "lib/libvector.h"

using namespace std;


int main()
{
	//rc_set_state(RUNNING);
	signal(SIGINT, __signal_handler);
	running = true;

	init_values_kf();
	init_sensors();

	while(running){
		printf("AINNN CIGARRINHUU");
		rc_usleep(100000);
		fflush(stdout);
	}

	return 0;
}
