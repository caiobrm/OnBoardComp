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

#define SAMPLES_LIMIT 10      // number of samples
#define DIFF_ALTITUDE_JUMP 25 // jump

// CONSTANTS //
#define GRAVITY 9.80665

// STATES //

#define STATE_STABILIZATION 0
#define STATE_PREPARED_4_FLIGHT 1
#define STATE_ACCELERATED_FLIGHT 2
#define STATE_RETARDED_FLIGHT 3
#define STATE_FALL_NO_PARACHUTE 4
#define STATE_FALL_PARACHUTE_DECELERATE 5
#define STATE_FALL_PARACHUTE_TERMINAL_VELOCITY 6
#define STATE_LANDED 7
#define STATE_UNKNOWN -1

// ACCEL FLAGS //

#define ACCEL_NEAR_ZERO 0
#define ACCEL_NEAR_G 1
#define ACCEL_HIGH_POSITIVE 2
#define ACCEL_LOW_NEGATIVE 3
#define ACCEL_HIGH_NEGATIVE 4 //never going to happen

// ALTITUDE FLAGS //

#define ALTITUDE_STATIONARY 0
#define ALTITUDE_RISING 1
#define ALTITUDE_FALLING 2

// PARACHUTE DETECTOR FLAGS //

#define PARACHUTE_DEACTIVATED 0
#define PARACHUTE_ACTIVATED 1

// STABILITY FLAGS  

#define STABILITY_ESTABLE 0
#define STABILITY_UNSTABLE 1

// CAM FLAGS //

#define CAM_ON 0
#define CAM_CAPTURE 1
#define CAM_ERROR 0xFF
#define CAM_OFF 3


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

extern int counter_ignitor;
extern int ignitionSignal;

#endif
