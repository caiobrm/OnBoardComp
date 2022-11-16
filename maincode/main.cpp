#include "lib/libvector.h"

using namespace std;

ofstream logs;

int main()
{
	rc_gpio_init(2, 3, GPIOHANDLE_REQUEST_OUTPUT);
	rc_gpio_set_value(2, 3, 0);

	rc_gpio_init(3, 1, GPIOHANDLE_REQUEST_INPUT);
	rc_gpio_init(3, 2, GPIOHANDLE_REQUEST_OUTPUT);
	rc_gpio_set_value(3, 2, 1);

	init_values_kf();
	running = init_sensors();

	int mytime = (int)time(NULL);
	char time_str[16];
	my_itoa(mytime, time_str);

	char path[50];
	strcpy(path, PATH);
	strcat(path, time_str);
	strcat(path, ".csv");

	logs.open(path, ios::app);

	headerLogging();

	long long unsigned int counter = 0;
	long long unsigned int initial_time;
	unsigned int n_iterations = 0;

	int counter_samples = 0;
	double leitura_anterior, leitura_nova;

	int caindo = 0;
	int paraquedas_acionado = 0;
	int counter_ignitor = 0;
	int sinal_acionamento = 0;

	while (running)
	{
		if (n_iterations == 0)
		{
			initial_time = rc_nanos_since_boot();
			leitura_nova = bmp_data.alt_m;
		}

		if (n_iterations % (BMP_RATE_DIV) == 0)
		{
			leitura_nova = bmp_data.alt_m;
			if (leitura_nova < leitura_anterior)
			{
				counter_samples++;
			}
			else
			{
				counter_samples = 0;
			}
			if (counter_samples >= 10)
			{
				caindo = 1;
				counter_samples = 0;
			}

			cout << "Valor novo: " << leitura_nova << "----- Valor antigo: " << leitura_anterior << "----- Caindo: " << caindo << "---- Counter Samples: " << counter_samples << "\n";
			leitura_anterior = leitura_nova;
		}

		// paraquedas_acionado = checkIgnitor();

		if (caindo == 1 && paraquedas_acionado == 0)
		{
			if (counter_ignitor < FS * TEMPO_ACIONAMENTO)
			{
				sinal_acionamento = 1;
				rc_gpio_set_value(2, 3, 1);
				counter_ignitor++;
			}
			else
			{
				rc_gpio_set_value(3, 2, 0);
				rc_gpio_set_value(2, 3, 0);

				counter_ignitor = 0;
				sinal_acionamento = 0;
				paraquedas_acionado = 1;
			}
		}

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
