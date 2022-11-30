
#include "lib/libvector.h"

using namespace std;

rc_mpu_data_t mpu_data;
rc_bmp_data_t bmp_data;
rc_kalman_t kf = RC_KALMAN_INITIALIZER;
rc_vector_t u = RC_VECTOR_INITIALIZER;
rc_vector_t y = RC_VECTOR_INITIALIZER;
rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
rc_mpu_config_t mpu_conf;
rc_matrix_t F = RC_MATRIX_INITIALIZER;
rc_matrix_t G = RC_MATRIX_INITIALIZER;
rc_matrix_t H = RC_MATRIX_INITIALIZER;
rc_matrix_t Q = RC_MATRIX_INITIALIZER;
rc_matrix_t R = RC_MATRIX_INITIALIZER;
rc_matrix_t Pi = RC_MATRIX_INITIALIZER;

char path[50];

long long unsigned int counter = 0;
long long unsigned int initial_time;
unsigned int n_iterations = 0;

int counter_samples = 0;
double oldData, newData;

bool falling = false;
bool parachuteOpen = false;
int counter_ignitor = 0;
int ignitionSignal = 0;

extern ofstream logs;

int checkIgnitor(void)
{
        return !rc_gpio_get_value(3, 1);
}

void console()
{
        printf("\r");
        // printf("%6.9lfs|", (double)counter / 1000000000);
        printf(" %8.4fm|", kf.x_est.d[0]);
        printf(" %7.4fm/s|", kf.x_est.d[1]);
        printf(" %7.4fm/s^2|", kf.x_est.d[2]);
        printf(" %9.4fm|", bmp_data.alt_m);
        printf(" %7.4fm/s^2|", acc_lp.newest_output);
        printf("\n");

        // Deve ser atualizada para cout em vez de printf
}

void logging()
{
        logs << setprecision(4) << fixed;

        logs << kf.x_est.d[0] << ",";
        logs << kf.x_est.d[1] << ",";
        logs << kf.x_est.d[2] << ",";
        logs << bmp_data.alt_m << ",";
        logs << acc_lp.newest_output;
        logs << "\n";
}

void create_path()
{
        int mytime = (int)time(NULL);
        char time_str[16];
        my_itoa(mytime, time_str);

        strcpy(path, PATH);
        strcat(path, time_str);
        strcat(path, ".csv");
}

void test_fall()
{
        if (n_iterations % (BMP_RATE_DIV) == 0)
        {
                newData = bmp_data.alt_m;
                if (newData < oldData)
                {
                        counter_samples++;
                }
                else
                {
                        counter_samples = 0;
                }
                if (counter_samples >= 10)
                {
                        falling = true;
                        counter_samples = 0;
                }

                cout << "Valor novo: " << newData << "----- Valor antigo: " << oldData << "----- falling: " << falling << "---- Counter Samples: " << counter_samples << "\n";
                oldData = newData;
        }
}

void parachute_triggering()
{
        if (falling == true && parachuteOpen == false)
        {
                if (counter_ignitor < FS * TEMPO_ACIONAMENTO)
                {
                        ignitionSignal = 1;
                        turnon_ledred();
                        counter_ignitor++;
                }
                else
                {
                        // rc_gpio_set_value(3, 2, 0);
                        // rc_gpio_set_value(2, 3, 0);

                        turnoff_ledred();
                        counter_ignitor = 0;
                        ignitionSignal = 0;
                        parachuteOpen = true;
                }
        }
}

void headerLogging()
{
        logs << "time,altitude,velocity,accel_bias,alt (bmp),vert_accel\n";
}

// PAUSE SECTION //
void ledState()
{
        if (rc_get_state() != EXITING)
        {
                turnon_ledgreen();
        }
}

char *my_itoa(int num, char *str)
{
        if (str == NULL)
        {
                return NULL;
        }
        sprintf(str, "%d", num);
        return str;
}