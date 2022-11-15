#include "lib/libvector.h"

using namespace std;

ofstream logs;

int main()
{
	//rc_set_state(RUNNING);
	running = true;

	init_values_kf();
	init_sensors();
	logs.open("../logs/logs.txt", ios::app);

	while(running){
		printf("1\n");
		headerLogging();
		rc_usleep(1000000);
		fflush(stdout);
	}
	logs.close();
	return 0;
}
