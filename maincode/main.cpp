#include "lib/libvector.h"

using namespace std;

ofstream logs;
// char path[50];

int main()
{
	init_gpios();

	init_values_kf();
	running = init_sensors();

	create_path();
	logs.open(path, ios::app);
	headerLogging();

	// initialize pause button
	if (rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
					   RC_BTN_DEBOUNCE_DEFAULT_US))
	{
		fprintf(stderr, "ERROR: failed to initialize pause button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, on_pause_press, on_pause_release);

	if (rc_get_state() != EXITING)
	{
		turnon_ledgreen();
	}

	while (rc_get_state() != EXITING)
	{
		if (n_iterations == 0)
		{
			initial_time = rc_nanos_since_boot();
			leitura_nova = bmp_data.alt_m;
		}

		test_fall();
		// paraquedas_acionado = checkIgnitor();

		parachute_triggering();

		cout << "Caindo: " << caindo << "---- Paraquedas_acionado: " << paraquedas_acionado << "----- SINAL_TESTE: " << sinal_acionamento << "------ counter_ignitor: " << counter_ignitor << "\n";

		logging();
		fflush(stdout);

		rc_nanosleep(1000000000 / FS - (rc_nanos_since_boot() - initial_time + 1185 - counter));
		counter = rc_nanos_since_boot() - initial_time + 1185;

		n_iterations++;
	}

	rc_mpu_power_off();
	rc_bmp_power_off();

	logs.close();
	return 0;
}
