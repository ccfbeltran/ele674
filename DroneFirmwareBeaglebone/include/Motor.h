/*
 * Motor.h
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <stdio.h>   /* Standard input/output definitions */
#include <stdlib.h>
#include <stdint.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <signal.h>
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <pthread.h>
#include <semaphore.h>
#include <sys/time.h>
#include <gpiod.h>


//#define POLICY SCHED_RR
#define POLICY SCHED_FIFO
#define THREADSTACK  65536

#define MOTOR_PERIOD	1

enum { MOTOR_NONE, MOTOR_PWM_ONLY, MOTOR_LED_ONLY, MOTOR_PWM_LED };

#define MOTOR_LEDOFF 0x0000
#define MOTOR_LEDRED 0x0100
#define MOTOR_LEDGREEN 0x0001
#define MOTOR_LEDORANGE 0x0101

#define MOTOR_UART "/dev/ttyS1"

#define GPIO_M1 57		//	GPIO01_25
#define GPIO_M2 49		//	GPIO01_17
#define GPIO_M3 116		//	GPIO03_20
#define GPIO_M4 113		//	GPIO03_17

#define GPIO_ERROR_READ 176		//	GPIO05_16
#define GPIO_ERROR_RESET 175	//	GPIO05_15

#define NUM_GPIO_CHIPS 			4
#define NUM_GPIO_LINE_PER_CHIP	32

struct gpiod_chip *gpio_chips[NUM_GPIO_CHIPS];
struct gpiod_line *gpio_lines[NUM_GPIO_CHIPS][NUM_GPIO_LINE_PER_CHIP];


typedef struct motor_struct {
	uint16_t	pwm[4];   //motor speed 0x00-0x1ff
	uint16_t	led[4];
	int			file;
	pthread_t 	MotorThread;
	pthread_spinlock_t 	MotorLock;
} MotorStruct;

int MotorInit (MotorStruct *Motor);
int MotorStart (void);
int MotorStop (MotorStruct *Motor);

#endif /* MOTOR_H_ */
