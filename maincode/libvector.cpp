#include "lib/libvector.h"

using namespace std;

FILE *fp;
bool running = false;

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
extern ofstream logs;

void init_values_kf()
{

        rc_matrix_zeros(&F, Nx, Nx);
        rc_matrix_zeros(&G, Nx, Nu);
        rc_matrix_zeros(&H, Ny, Nx);
        rc_matrix_zeros(&Q, Nx, Nx);
        rc_matrix_zeros(&R, Ny, Ny);
        rc_matrix_zeros(&Pi, Nx, Nx);
        rc_vector_zeros(&u, Nu);
        rc_vector_zeros(&y, Ny);

        F.d[0][0] = 1.0;
        F.d[0][1] = DT;
        F.d[0][2] = 0.0;
        F.d[1][0] = 0.0;
        F.d[1][1] = 1.0;
        F.d[1][2] = -DT; // subtract accel bias
        F.d[2][0] = 0.0;
        F.d[2][1] = 0.0;
        F.d[2][2] = 1.0; // accel bias state
        G.d[0][0] = 0.5 * DT * DT;
        G.d[0][1] = DT;
        G.d[0][2] = 0.0;
        H.d[0][0] = 1.0;
        H.d[0][1] = 0.0;
        H.d[0][2] = 0.0;
        // covariance matrices
        Q.d[0][0] = 0.000000001;
        Q.d[1][1] = 0.000000001;
        Q.d[2][2] = 0.0001; // don't want bias to change too quickly
        R.d[0][0] = 1000000.0;
        // initial P, cloned from converged P while running
        Pi.d[0][0] = 1258.69;
        Pi.d[0][1] = 158.6114;
        Pi.d[0][2] = -9.9937;
        Pi.d[1][0] = 158.6114;
        Pi.d[1][1] = 29.9870;
        Pi.d[1][2] = -2.5191;
        Pi.d[2][0] = -9.9937;
        Pi.d[2][1] = -2.5191;
        Pi.d[2][2] = 0.3174;
}

bool init_sensors()
{
        signal(SIGINT, __signal_handler);
        if (rc_kalman_alloc_lin(&kf, F, G, H, Q, R, Pi) == -1)
                return false;
        // initialize the little LP filter to take out accel noise
        if (rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC))
                return false;
        // set signal handler so the loop can exit cleanly
        // signal(SIGINT, __signal_handler);
        running = true;
        // init barometer and read in first data
        printf("initializing barometer\n");
        if (rc_bmp_init(BMP_OVERSAMPLE_16, BMP_FILTER_16))
                return false;
        if (rc_bmp_read(&bmp_data))
                return false;
        // init DMP
        printf("initializing DMP\n");
        mpu_conf = rc_mpu_default_config();
        mpu_conf.dmp_sample_rate = SAMPLE_RATE;
        mpu_conf.dmp_fetch_accel_gyro = 1;
        if (rc_mpu_initialize_dmp(&mpu_data, mpu_conf))
                return false;
        // wait for dmp to settle then start filter callback
        printf("waiting for sensors to settle");
        fflush(stdout);
        rc_usleep(3000000);
        rc_mpu_set_dmp_callback(__dmp_handler);
        return true;
}

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

void headerLogging()
{
        logs << "time,altitude,velocity,accel_bias,alt (bmp),vert_accel\n";
}

void __signal_handler(__attribute__((unused)) int dummy)
{
        running = false;
        return;
}
void __dmp_handler(void)
{
        int i;
        double accel_vec[3];
        static int bmp_sample_counter = 0;
        // make copy of acceleration reading before rotating
        for (i = 0; i < 3; i++)
                accel_vec[i] = mpu_data.accel[i];
        // rotate accel vector
        rc_quaternion_rotate_vector_array(accel_vec, mpu_data.dmp_quat);
        // do first-run filter setup
        if (kf.step == 0)
        {
                kf.x_est.d[0] = bmp_data.alt_m;
                rc_filter_prefill_inputs(&acc_lp, accel_vec[2] - 9.80665);
                rc_filter_prefill_outputs(&acc_lp, accel_vec[2] - 9.80665);
        }
        // calculate acceleration and smooth it just a tad
        rc_filter_march(&acc_lp, accel_vec[2] - 9.80665);
        u.d[0] = acc_lp.newest_output;
        // don't bother filtering Barometer, kalman will deal with that
        y.d[0] = bmp_data.alt_m;
        if (rc_kalman_update_lin(&kf, u, y))
                running = false;
        // now check if we need to sample BMP this loop
        bmp_sample_counter++;
        if (bmp_sample_counter >= BMP_RATE_DIV)
        {
                // perform the i2c reads to the sensor, on bad read just try later
                if (rc_bmp_read(&bmp_data))
                        return;
                bmp_sample_counter = 0;
        }
        return;
}

// PAUSE SECTION //

void on_pause_release()
{
        if (rc_get_state() == RUNNING)
                rc_set_state(PAUSED);
        else if (rc_get_state() == PAUSED)
                rc_set_state(RUNNING);
        return;
}

/**
 * If the user holds the pause button for 2 seconds, set state to EXITING which
 * triggers the rest of the program to exit cleanly.
 **/
void on_pause_press()
{
        int i;
        const int samples = 100;     // check for release 100 times in this period
        const int us_wait = 2000000; // 2 seconds

        // now keep checking to see if the button is still held down
        for (i = 0; i < samples; i++)
        {
                rc_usleep(us_wait / samples);
                if (rc_button_get_state(RC_BTN_PIN_PAUSE) == RC_BTN_STATE_RELEASED)
                        return;
        }
        printf("long press detected, shutting down\n");
        rc_set_state(EXITING);
        return;
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