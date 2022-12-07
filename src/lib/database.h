#pragma once

#include <stdio.h>
#include <iostream>
#include <robotcontrol.h>
#include <time.h>
#include <signal.h>
#include <fstream>
#include <iomanip>

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

#define FS 50 // hz

#define SAMPLES_LIMIT 10 // number of samples

// VARIABLES //

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

extern char path[50];

extern long long unsigned int counter;
extern long long unsigned int initial_time;
extern unsigned int n_iterations;

extern int counter_samples_fall;
extern int counter_samples_rise;
extern double oldData, newData;

extern bool falling;
extern bool rising;
extern bool stationary;

extern bool parachuteOpen;
extern int counter_ignitor;
extern int ignitionSignal;

#endif
