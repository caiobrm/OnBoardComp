#include "lib/libvector.h"

using namespace std;

ofstream logs;

// char path[50];

int main()
{
	uint8_t state = STATE_STABILIZATION, accel_flag = ACCEL_NEAR_ZERO, altitude_flag = ALTITUDE_STATIONARY;

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

		check_barometer(&altitude_flag);
		check_accel(&accel_flag);
		// parachuteOpen = checkIgnitor();

		//parachute_triggering();

		// cout << "falling: " << falling << "---- rising " << rising << "----- stationary: " << stationary << "\n";

		logging();
		fflush(stdout);

		rc_nanosleep(1000000000 / FS - (rc_nanos_since_boot() - initial_time + 1185 - counter)); //error if 0
		counter = rc_nanos_since_boot() - initial_time + 1185;

		n_iterations++;
	}

	turnoff_ledgreen();

	rc_mpu_power_off();
	rc_bmp_power_off();

	logs.close();
	return 0;
}
