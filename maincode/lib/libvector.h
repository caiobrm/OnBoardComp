#include <stdio.h>
#include <iostream>
#include <robotcontrol.h>
#include "database.h"
#include <time.h>
#include <math.h>
#include <signal.h>

#ifndef LIBVECTOR_H_INCLUDED
#define LIBVECTOR_H_INCLUDED


void init_values_kf();
int init_sensors();


void console();
int checkIgnitor();
void headerbb();
void headerll();


void on_pause_press();
void on_pause_release();
void __signal_handler(__attribute__ ((unused)) int dummy);
void __dmp_handler(void);



#endif