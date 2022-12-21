
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

int counter_samples_fall = 0;
int counter_samples_rise = 0;
double oldData, newData;

int counter_ignitor = 0;
int ignitionSignal = 0;

extern ofstream logs;

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
        logs << (double)counter / 1000000000;
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

void check_barometer(uint8_t * altitude_flag)
{
      
        if (n_iterations % (BMP_RATE_DIV) == 0)
        {
                newData = bmp_data.alt_m;
                if (newData <= oldData - DIFF_ALTITUDE_JUMP)
                {
                        counter_samples_fall += 3;
                }
                else if (newData < oldData)
                {
                        counter_samples_fall++;
                }
                else
                {
                        counter_samples_fall = 0;
                }
                if (newData >= oldData + DIFF_ALTITUDE_JUMP)
                {
                        counter_samples_rise += 3;
                }
                else if (newData > oldData)
                {
                        counter_samples_rise++;
                }
                else
                {
                        counter_samples_rise = 0;
                }
                if (counter_samples_fall >= SAMPLES_LIMIT)
                {
                        *altitude_flag = ALTITUDE_FALLING;
                        counter_samples_fall = 0;
                }
                else if (counter_samples_rise >= SAMPLES_LIMIT)
                {
                        *altitude_flag = ALTITUDE_RISING;
                        counter_samples_rise = 0;
                }
                else
                {
                        *altitude_flag = ALTITUDE_STATIONARY;
                }

                cout << "Valor novo: " << newData << "----- Valor antigo: " << oldData << "----- altitude_flag: " << unsigned(*altitude_flag) << "---- CS_fall: " << counter_samples_fall << "--- CS_rise  " << counter_samples_rise << "\n";
                oldData = newData;
        }
}

void check_accel(uint8_t * accel_flag)
{
        if(acc_lp.newest_output <= 1 && acc_lp.newest_output >= -1)
        {
                *accel_flag = ACCEL_NEAR_ZERO;
        }
        else if(acc_lp.newest_output <= -GRAVITY + 1 && acc_lp.newest_output >= -GRAVITY-1)
        {
                *accel_flag = ACCEL_NEAR_G;
        }
        else if(acc_lp.newest_output > 1)
        {
                *accel_flag = ACCEL_HIGH_POSITIVE;
        }
        else if(acc_lp.newest_output <= -1 && acc_lp.newest_output >= -GRAVITY+1)
        {
                *accel_flag = ACCEL_LOW_NEGATIVE;
        }
        else
        {
                *accel_flag = ACCEL_HIGH_NEGATIVE;
        }

        cout << "accel_flag: " << unsigned(*accel_flag) << "\n";
}

void check_parachute(uint8_t * parachute_flag)
        


/*
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
*/

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