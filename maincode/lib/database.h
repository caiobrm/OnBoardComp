#include <robotcontrol.h>

#ifndef DATABASE_H_INCLUDED
#define DATABASE_H_INCLUDED

#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE 200 // hz
#define DT (1.0 / SAMPLE_RATE)
#define ACCEL_LP_TC 20 * DT // fast LP filter for accel
#define PRINT_HZ 10
#define BMP_RATE_DIV 10 // optionally sample bmp less frequently than mpu

#define FS              50     //hz

// VARIABLES //
extern bool running;
extern FILE *fp;


extern rc_mpu_data_t mpu_data;
extern rc_bmp_data_t bmp_data;
extern rc_kalman_t kf;
extern rc_vector_t u;
extern rc_vector_t y;
extern rc_filter_t acc_lp;
extern rc_mpu_config_t mpu_conf;
extern rc_matrix_t F;
extern rc_matrix_t G;
extern rc_matrix_t H;
extern rc_matrix_t Q;
extern rc_matrix_t R;
extern rc_matrix_t Pi;


#endif
