/*
 * Motor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */


#include "Motor.h"

#define TimeDelay 50000

uint16_t GPIO_TAB[4] = {GPIO_M1, GPIO_M2, GPIO_M3, GPIO_M4};

extern sem_t	MotorTimerSem;
extern int		MotorActivated;

pthread_barrier_t 	MotorStartBarrier;

int gpio_set (int gpio_line, int val)  {
	int	chip = (gpio_line / NUM_GPIO_LINE_PER_CHIP);
	int	line = (gpio_line % NUM_GPIO_LINE_PER_CHIP);
	char gpio_name[20];

	if (gpio_chips[chip] == NULL) {
		gpio_chips[chip] = gpiod_chip_open_by_number(chip);
		if (gpio_chips[chip] == NULL) {
			printf("%s : %s -> Error : cannot open GPIO CHIP (%u)\n", __FILE__, __FUNCTION__, chip);
		}
	}
	if (gpio_lines[chip][line] == NULL) {
		gpio_lines[chip][line] = gpiod_chip_get_line(gpio_chips[chip], line);
		if (gpio_lines[chip][line] == NULL) {
			printf("%s : %s -> Error : cannot open GPIO LINE (%u : %u = %u)\n", __FILE__, __FUNCTION__, chip, line, gpio_line);
		}
		sprintf(gpio_name, "DroneMotorGPIO%u", gpio_line);
		gpiod_line_request_output(gpio_lines[chip][line], gpio_name, GPIOD_LINE_ACTIVE_STATE_HIGH);
	}
	return gpiod_line_set_value(gpio_lines[chip][line], val);
}


int motor_open(void) {
	struct termios config;
	int uart = open(MOTOR_UART, O_RDWR | O_NOCTTY | O_NDELAY);

	if (uart < 0) {
		printf("motor_open : impossible d'ouvrir le uart du moteur\n");
		return uart;
	}

	fcntl(uart, F_SETFL, 0); //read calls are non blocking

	//set port config
	tcgetattr(uart, &config);
	cfsetspeed(&config, B115200);
	config.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	config.c_iflag = 0; //clear input config
	config.c_lflag = 0; //clear local config
	config.c_oflag &= ~OPOST; //clear output config (raw output)
	cfmakeraw(&config);
	tcsetattr(uart, TCSANOW, &config);
	return uart;
}

int motor_cmd(int file, uint8_t cmd, uint8_t *reply, int replylen) {
	int size;

	write(file, &cmd, 1);
	fsync(file);
	usleep(TimeDelay);
	size = read(file, reply, replylen);

	return size;
}

int MotorPortInit(MotorStruct *Motor) {
	uint8_t reply[256];
	int		i;

	//open motor port
	Motor->file = motor_open();
	if (Motor->file < 0) {
		printf("motor_open: Impossible d'ouvrir le UART\n");
		return Motor->file;
	}

	//reset IRQ flipflop - this code resets GPIO_ERROR_READ to 0
	//all select lines inactive
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_TAB[i], 0);
	}
	usleep(2*TimeDelay);

	//configure motors
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_TAB[i], 1);
		usleep(2*TimeDelay);
		motor_cmd(Motor->file, 0xE0, reply, 2);
		motor_cmd(Motor->file, 0x91, reply, 121);
		motor_cmd(Motor->file, 0xA1, reply, 2);
		motor_cmd(Motor->file, i + 1, reply, 1);
		motor_cmd(Motor->file, 0x40, reply, 2);
		gpio_set(GPIO_TAB[i], 0);
		usleep(2*TimeDelay);
	}

	//all select lines active
	for (i = 0; i < 4; ++i) {
		gpio_set(GPIO_TAB[i], 1);
	}
	usleep(2*TimeDelay);

	return 0;
}


void motor_send(MotorStruct *Motor, int SendMode) {
/* Fonction utilitaire pour simplifier les transmissions aux moteurs */

	switch (SendMode) {
	case MOTOR_NONE : 		break;
	case MOTOR_PWM_ONLY :	/* A faire! */
							break;
	case MOTOR_LED_ONLY :	/* A faire! */
							break;
	case MOTOR_PWM_LED :	/* A faire! */
							break;
	}
}


void *MotorTask ( void *ptr ) {
/* A faire! */
/* Tache qui transmet les nouvelles valeurs de vitesse */
/* à chaque moteur à interval régulier (5 ms).         */
	while (MotorActivated) {
//		DOSOMETHING();
	}
	pthread_exit(0); /* exit thread */
}


int MotorInit (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation des moteurs.   */
/* C'est-à-dire initialiser le Port des moteurs avec la    */
/* fonction MotorPortInit() et créer la Tâche MotorTask()  */
/* qui va s'occuper des mises à jours des moteurs en cours */ 
/* d'exécution.                                            */
	return 0;
}



int MotorStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer la mise à jour des moteurs (MotorTask).    */ 
/* Tout le système devrait être prêt à faire leur travail et il ne reste */
/* plus qu'à tout démarrer.                                              */
	return retval;
}



int MotorStop (MotorStruct *Motor) {
/* A faire! */
/* Ici, vous devriez arrêter les moteurs et fermer le Port des moteurs. */ 
	return 0;
}

