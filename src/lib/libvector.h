#pragma once
#include "database.h"

#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED

#define BMP_RATE_DIV 10     // optionally sample bmp less frequently than mpu
#define FS 50               // hz
#define TEMPO_ACIONAMENTO 2 // em segundos

#define PATH "/OnBoardComp/data/"

void init_values_kf();
void init_sensors();

void console();
void logging();
void init_gpios();
void turnon_ledgreen();
void turnoff_ledgreen();
void turnon_ledred();
void turnoff_ledred();
void create_path();
void check_barometer(uint8_t *);
void check_accel();
//void parachute_triggering();
int checkIgnitor();
void headerbb();
void headerll();
void headerLogging();
void pauseButton();
void ledState();

void on_pause_press();
void on_pause_release();
void __signal_handler(__attribute__((unused)) int dummy);
void __dmp_handler(void);
char *my_itoa(int, char *);

#endif