#include "lib/libvector.h"

using namespace std;

ofstream logs;

int main()
{
	//rc_set_state(RUNNING);
	running = true;

	init_values_kf();
	init_sensors();

	logs.open(PATH, ios::app);

	while(running){


		headerLogging();
		rc_usleep(1000000);
		fflush(stdout);

	}
	logs.close();
	return 0;
}
