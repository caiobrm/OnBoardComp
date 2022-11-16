
#include "database.h"

#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED

#define BMP_RATE_DIV 10     // optionally sample bmp less frequently than mpu
#define FS 50               // hz
#define TEMPO_ACIONAMENTO 2 // em segundos

#define PATH "/cplusplus/OnBoardComp/data/"

void init_values_kf();
bool init_sensors();

void console();
void logging();
void init_gpios();
void create_path();
void test_fall();
void parachute_triggering();
int checkIgnitor();
void headerbb();
void headerll();
void headerLogging();

void on_pause_press();
void on_pause_release();
void __signal_handler(__attribute__((unused)) int dummy);
void __dmp_handler(void);
char *my_itoa(int, char *);
#endif