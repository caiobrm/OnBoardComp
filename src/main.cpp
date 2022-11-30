#include "lib/libvector.h"

using namespace std;

ofstream logs;
// char path[50];

int main()
{
	init_gpios();

	init_values_kf();
	init_sensors();

	create_path();
	logs.open(path, ios::app);
	headerLogging();

	pauseButton();
	// ledState();

	while (rc_get_state() != EXITING)
	{
		turnon_ledgreen();

		if (n_iterations == 0)
		{
			initial_time = rc_nanos_since_boot();
			newData = bmp_data.alt_m;
		}

		test_fall();
		// parachuteOpen = checkIgnitor();

		parachute_triggering();

		cout << "falling: " << falling << "---- parachuteOpen: " << parachuteOpen << "----- SINAL_TESTE: " << ignitionSignal << "------ counter_ignitor: " << counter_ignitor << "\n";

		logging();
		fflush(stdout);

		rc_nanosleep(1000000000 / FS - (rc_nanos_since_boot() - initial_time + 1185 - counter));
		counter = rc_nanos_since_boot() - initial_time + 1185;

		n_iterations++;
	}

	turnoff_ledgreen();

	rc_mpu_power_off();
	rc_bmp_power_off();

	logs.close();
	return 0;
}
