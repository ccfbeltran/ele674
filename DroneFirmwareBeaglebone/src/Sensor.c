/*
 * Sensor.c
 *
 *  Created on: 12 sept. 2013
 *      Author: bruno
 */

#include "Sensor.h"


#define ABS(x) (((x) < 0.0) ? -(x) : (x))


#define MAX_TOT_SAMPLE 1000

pthread_barrier_t   SensorStartBarrier;
pthread_barrier_t   LogStartBarrier;
pthread_mutex_t 	Log_Mutex;

uint8_t  SensorsActivated 	= 0;
uint8_t  LogActivated  	  	= 0;
uint8_t  numLogOutput 	  	= 0;
uint8_t  init_gyro 			= 0;

static double coordonne_mag		= 0.0;
static double mag_range			= 0.0;

static int last_idx_mag 		= 0;
static int last_idx_sonar 		= 0;
static int last_idx_accel 		= 0;
static int last_idx_baro 		= 0;

static double borne_accel_max_x 	= 0.0;
static double borne_accel_max_y 	= 0.0;
static double borne_accel_max_z 	= 0.0;

void Detection_erreur(SensorStruct *Sensor){

	double norm = 0;

	switch(Sensor->type){
		case SONAR:
			//Si la valeur final presente est superieur que 6.118 metre, prend l'ancienne valeur valid
				if(Sensor->Data[Sensor->DataIdx].Data[0] > SONAR_MAX_METER){
					Sensor->Data[Sensor->DataIdx].Data[0] = Sensor->Data[last_idx_sonar].Data[0];
				}
				else{
					last_idx_sonar = Sensor->DataIdx;
				}

		break;

		case MAGNETOMETRE:
				//norme de x,y,z
				norm = sqrt(Sensor->Data[Sensor->DataIdx].Data[0]*Sensor->Data[Sensor->DataIdx].Data[0]
				+Sensor->Data[Sensor->DataIdx].Data[1]*Sensor->Data[Sensor->DataIdx].Data[1]
				+Sensor->Data[Sensor->DataIdx].Data[2]*Sensor->Data[Sensor->DataIdx].Data[2]);

				coordonne_mag += norm; //accumulation des norme

				//On a recueilli les 100 valeur
				if(Sensor->DataIdx  == DATABUFSIZE-1){
					coordonne_mag /= DATABUFSIZE; //moyenne

					if (mag_range < coordonne_mag) //Remplace si la nouvelle moyenne est superieur que l'ancienne borne
					{
						mag_range = coordonne_mag;
					}
					coordonne_mag = 0.0;
				}
				//Si la norme presente est superieur que la borne, prend l'ancienne valeur valid
				if(norm > mag_range){
					Sensor->Data[Sensor->DataIdx].Data[0] = Sensor->Data[last_idx_sonar].Data[0];
					Sensor->Data[Sensor->DataIdx].Data[1] = Sensor->Data[last_idx_sonar].Data[1];
					Sensor->Data[Sensor->DataIdx].Data[2] = Sensor->Data[last_idx_sonar].Data[2];
				}
				else{
					last_idx_sonar = Sensor->DataIdx;
				}

		break;
		case ACCELEROMETRE:

			//Initialisation des bornes avec la calibration
			if(borne_accel_max_x == 0 && borne_accel_max_y == 0 && borne_accel_max_z == 0){
					borne_accel_max_x = (Sensor->Param->alpha[0][0]*LIMITE_ACCEL_MS2) + Sensor->Param->beta[0]; //alpha_x*max_speed + beta_x
					borne_accel_max_y = (Sensor->Param->alpha[1][1]*LIMITE_ACCEL_MS2) + Sensor->Param->beta[1]; //alpha_y*max_speed + beta_y
					borne_accel_max_z = (Sensor->Param->alpha[2][2]*LIMITE_ACCEL_MS2) + Sensor->Param->beta[2]; //alpha_z*max_speed + beta_z
			}
			//Si les valeur finale presente est superieur que la borne, prend l'ancienne valeur valid
			if(abs(Sensor->Data[Sensor->DataIdx].Data[0]) > borne_accel_max_x ||
					abs(Sensor->Data[Sensor->DataIdx].Data[1]) > borne_accel_max_y ||
					abs(Sensor->Data[Sensor->DataIdx].Data[2]) > borne_accel_max_z)
			{
					Sensor->Data[Sensor->DataIdx].Data[0] = Sensor->Data[last_idx_accel].Data[0];
					Sensor->Data[Sensor->DataIdx].Data[1] = Sensor->Data[last_idx_accel].Data[1];
					Sensor->Data[Sensor->DataIdx].Data[2] = Sensor->Data[last_idx_accel].Data[2];
			}
			else{
				last_idx_accel = Sensor->DataIdx;
			}

		break;
	}

}


void *SensorTask ( void *ptr ) {
/* A faire! */
/* Tache qui sera instancié pour chaque sensor. Elle s'occupe d'aller */
/* chercher les donnees du sensor.                                    */
	uint32_t retval;
	SensorRawData tampon_raw_data;
	uint64_t time_stamp_avant=0;

	pthread_barrier_wait(&(SensorStartBarrier));
	/*on transforme notre pointeur vide en pointer SensorRawData*/
	SensorStruct *Sensor = (SensorStruct *)ptr;

	while (SensorsActivated) {

		/*lire la donne du capteur et le garder dans sonr_raw_data*/

		read(Sensor->File,&tampon_raw_data,sizeof(SensorRawData));

		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		Sensor->DataIdx++;
		Sensor->DataIdx=Sensor->DataIdx % DATABUFSIZE;
		pthread_spin_lock(&(Sensor->DataLock));
		Sensor->RawData[Sensor->DataIdx]=tampon_raw_data;

		if(Sensor->type == SONAR || Sensor->type == BAROMETRE){ //pas de calibration

			Sensor->Data[Sensor->DataIdx].Data[0] = 0;

			for(int i = 0; i < 3; i++){
				Sensor->Data[Sensor->DataIdx].Data[0] += Sensor->Param->alpha[0][i]*(Sensor->RawData[Sensor->DataIdx].data[i]*Sensor->Param->Conversion);
			}

			Sensor->Data[Sensor->DataIdx].Data[0] += Sensor->Param->beta[0];
		}
		//Calibration du Gyroscope
		else if(Sensor->type == GYROSCOPE && init_gyro < 100){

			if(Sensor->RawData[Sensor->DataIdx].data[0] > 0)
				Sensor->RawData[Sensor->DataIdx].data[0] = -Sensor->RawData[Sensor->DataIdx].data[0];

			if(Sensor->RawData[Sensor->DataIdx].data[1] > 0)
				Sensor->RawData[Sensor->DataIdx].data[1] = -Sensor->RawData[Sensor->DataIdx].data[1];

			if(Sensor->RawData[Sensor->DataIdx].data[2] > 0)
				Sensor->RawData[Sensor->DataIdx].data[2] = -Sensor->RawData[Sensor->DataIdx].data[2];

			Sensor->Param->beta[0] += Sensor->RawData[Sensor->DataIdx].data[0];
			Sensor->Param->beta[1] += Sensor->RawData[Sensor->DataIdx].data[1];
			Sensor->Param->beta[2] += Sensor->RawData[Sensor->DataIdx].data[2];

			if(init_gyro == 99){

				Sensor->Param->beta[0] /= 100;
				Sensor->Param->beta[1] /= 100;
				Sensor->Param->beta[2] /= 100;

				printf("Calibration du Gyroscope done \n");
			}

			init_gyro++;
		}
		else{ //calibration des autres capteurs
			 // x', y', z' a zero
			Sensor->Data[Sensor->DataIdx].Data[0] = 0;
			Sensor->Data[Sensor->DataIdx].Data[1] = 0;
			Sensor->Data[Sensor->DataIdx].Data[2] = 0;

			//Calcul la matrice alpha avec x,y,z. ------> x, y, z = Rawdata*conversion
			for(int i = 0; i < 3; i++){
				Sensor->Data[Sensor->DataIdx].Data[0] += Sensor->Param->alpha[0][i]*(Sensor->RawData[Sensor->DataIdx].data[i]*Sensor->Param->Conversion);
				Sensor->Data[Sensor->DataIdx].Data[1] += Sensor->Param->alpha[1][i]*(Sensor->RawData[Sensor->DataIdx].data[i]*Sensor->Param->Conversion);
				Sensor->Data[Sensor->DataIdx].Data[2] += Sensor->Param->alpha[2][i]*(Sensor->RawData[Sensor->DataIdx].data[i]*Sensor->Param->Conversion);
			}
			//x', y', z' + Beta
			Sensor->Data[Sensor->DataIdx].Data[0] += Sensor->Param->beta[0];
			Sensor->Data[Sensor->DataIdx].Data[1] += Sensor->Param->beta[1];
			Sensor->Data[Sensor->DataIdx].Data[2] += Sensor->Param->beta[2];
		}

		Detection_erreur(Sensor);
		/*on ajuste le time stamp*/
		Sensor->Data[Sensor->DataIdx].TimeDelay= tampon_raw_data.timestamp-time_stamp_avant;
		/*on sauvegarde la valeur precedante*/
		time_stamp_avant=tampon_raw_data.timestamp;

		pthread_spin_unlock(&(Sensor->DataLock));

		pthread_cond_broadcast(&(Sensor->DataNewSampleCondVar));
		pthread_mutex_unlock(&(Sensor->DataSampleMutex));
	}
	pthread_exit(0); /* exit thread */
}


int SensorsInit (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez faire l'initialisation de chacun des capteurs.  */ 
/* C'est-à-dire de faire les initialisations requises, telles que    */
/* ouvrir les fichiers des capteurs, et de créer les Tâches qui vont */
/* s'occuper de réceptionner les échantillons des capteurs.          */
//

	// { ACCEL_INIT, GYRO_INIT, MAGNETO_INIT, BAROM_INIT, SONAR_INIT };

	int retval = 0;
	pthread_barrier_init(&SensorStartBarrier, NULL, NUM_SENSOR + 1);
	for(int i =0; i < NUM_SENSOR; i++)
	{
		pthread_mutex_init(&(SensorTab[i].DataSampleMutex), NULL);
		pthread_spin_init(&(SensorTab[i].DataLock), PTHREAD_PROCESS_PRIVATE);
		pthread_cond_init(&(SensorTab[i].DataNewSampleCondVar), NULL);

		SensorTab[i].File = open(SensorTab[i].DevName, O_RDONLY);

		if(SensorTab[i].File < 0){
			printf("Open capteur %d and DevName %s: Impossible douvrir\n", i, SensorTab[i].DevName);
			return -1;
		}

		printf("Open capteur %d succes\n", i);
		retval = pthread_create(&(SensorTab[i].SensorThread), NULL, SensorTask, (void*) &(SensorTab[i]));

		if(retval){
			printf("pthread_create : Impossible le thread %d\n", i);
			return retval;
		}
	}

	printf("SensorsInit succes\n");

	return retval;
};


int SensorsStart (void) {
/* A faire! */
/* Ici, vous devriez démarrer l'acquisition sur les capteurs.        */ 
/* Les capteurs ainsi que tout le reste du système devrait être      */
/* prêt à faire leur travail et il ne reste plus qu'à tout démarrer. */

	SensorsActivated = 1;
	pthread_barrier_wait(&(SensorStartBarrier));
	pthread_barrier_destroy(&(SensorStartBarrier));
	printf("%s Sensors démarré\n", __FUNCTION__);

	return 0;
}


int SensorsStop (SensorStruct SensorTab[NUM_SENSOR]) {
/* A faire! */
/* Ici, vous devriez défaire ce que vous avez fait comme travail dans */
/* SensorsInit() (toujours verifier les retours de chaque call)...    */ 

	int err = 0;

	SensorsActivated = 0;

	for(int i = 0; i < NUM_SENSOR; i++){
		err	= pthread_join(SensorTab[i].SensorThread, NULL);

		if(err){
			printf("pthread_join(SensorThread[%d]) : Erreur\n", i);
			return err;
		}
		pthread_mutex_destroy(&(SensorTab[i].DataSampleMutex));
		pthread_spin_destroy(&(SensorTab[i].DataLock));
		pthread_cond_destroy(&(SensorTab[i].DataNewSampleCondVar));
	}

	return err;
}



/* Le code ci-dessous est un CADEAU !!!	*/
/* Ce code permet d'afficher dans la console les valeurs reçues des capteurs.               */
/* Évidemment, celà suppose que les acquisitions sur les capteurs fonctionnent correctement. */
/* Donc, dans un premier temps, ce code peut vous servir d'exemple ou de source d'idées.     */
/* Et dans un deuxième temps, peut vous servir pour valider ou vérifier vos acquisitions.    */
/*                                                                                           */
/* NOTE : Ce code suppose que les échantillons des capteurs sont placés dans un tampon       */
/*        circulaire de taille DATABUFSIZE, tant pour les données brutes (RawData) que       */
/*        les données converties (NavData) (voir ci-dessous)                                 */

void *SensorLogTask ( void *ptr ) {
	SensorStruct	*Sensor    = (SensorStruct *) ptr;
	uint16_t		*Idx       = &(Sensor->DataIdx);
	uint16_t		LocalIdx   = DATABUFSIZE;
	SensorData	 	*NavData   = NULL;
	SensorRawData	*RawData   = NULL;
	SensorRawData   tpRaw;
	SensorData 	    tpNav;
	double			norm;

	printf("%s : Log de %s prêt à démarrer\n", __FUNCTION__, Sensor->Name);
	pthread_barrier_wait(&(LogStartBarrier));

	while (LogActivated) {
		pthread_mutex_lock(&(Sensor->DataSampleMutex));
		while ((LocalIdx == *Idx)&&(LogActivated != 0))
			pthread_cond_wait(&(Sensor->DataNewSampleCondVar), &(Sensor->DataSampleMutex));
		LocalIdx = *Idx;
	    pthread_mutex_unlock(&(Sensor->DataSampleMutex));
		if (LogActivated == 0) {
		    pthread_mutex_unlock(&(Sensor->DataSampleMutex));
			break;
		}

	   	pthread_spin_lock(&(Sensor->DataLock));
    	NavData   = &(Sensor->Data[LocalIdx]);
    	RawData   = &(Sensor->RawData[LocalIdx]);
		memcpy((void *) &tpRaw, (void *) RawData, sizeof(SensorRawData));
		memcpy((void *) &tpNav, (void *) NavData, sizeof(SensorData));
	   	pthread_spin_unlock(&(Sensor->DataLock));

	   	pthread_mutex_lock(&Log_Mutex);
		if (numLogOutput == 0)
			printf("Sensor  :     TimeStamp  SampleDelay  Status  SampleNum   Raw Sample Data  =>        Converted Sample Data               Norme\n");
		else switch (tpRaw.type) {
				case ACCELEROMETRE :	norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Accel   : (%09llu)-(%09u)  %2d     %8u   %d  %d  %d  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf, borne_x = %lf, borne_y = %lf, borne_z = %lf\n", tpRaw.timestamp, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, tpRaw.data[0], tpRaw.data[1], tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm, borne_accel_max_x, borne_accel_max_y, borne_accel_max_z);
										break;
				case GYROSCOPE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Gyro    : (%09llu)-(%09u)  %2d     %8u   %04X  %04X  %04X  =>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf\n", tpRaw.timestamp, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], (uint16_t) tpRaw.data[1], (uint16_t) tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm);
										break;
				case SONAR :			printf("Sonar   : (%09llu)-(%09u)  %2d     %8u   %d              =>  %10.5lf\n", tpRaw.timestamp, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, tpRaw.data[0], tpNav.Data[0]);
										break;
				case BAROMETRE :		printf("Barom   : (%09llu)-(%09u)  %2d     %8u   %04X              =>  %10.5lf\n", tpRaw.timestamp, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num, (uint16_t) tpRaw.data[0], tpNav.Data[0]);
										break;
				case MAGNETOMETRE :		norm = sqrt(tpNav.Data[0]*tpNav.Data[0]+tpNav.Data[1]*tpNav.Data[1]+tpNav.Data[2]*tpNav.Data[2]);
										printf("Magneto : (%09llu)-(%09u)  %2d     %8u   %d  %d   %d	=>  %10.5lf  %10.5lf  %10.5lf  =  %10.5lf, mag = %lf, coor = %lf\n", tpRaw.timestamp, tpNav.TimeDelay, tpRaw.status, tpRaw.ech_num,  tpRaw.data[0],  tpRaw.data[1],  tpRaw.data[2], tpNav.Data[0], tpNav.Data[1], tpNav.Data[2], norm, mag_range, coordonne_mag);
										break;
			 }
		numLogOutput++;
		if (numLogOutput > 20)
			numLogOutput = 0;
		pthread_mutex_unlock(&Log_Mutex);
	}

	printf("%s : %s Terminé\n", __FUNCTION__, Sensor->Name);

	pthread_exit(0); /* exit thread */
}


int InitSensorLog (SensorStruct *Sensor) {
	pthread_attr_t		attr;
	struct sched_param	param;
	int					retval;

	pthread_attr_init(&attr);
	pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_attr_setscope(&attr, PTHREAD_SCOPE_PROCESS);
	pthread_attr_setschedpolicy(&attr, POLICY);
	param.sched_priority = sched_get_priority_min(POLICY);
	pthread_attr_setstacksize(&attr, THREADSTACK);
	pthread_attr_setschedparam(&attr, &param);

	printf("Creating Log thread : %s\n", Sensor->Name);
	if ((retval = pthread_create(&(Sensor->LogThread), &attr, SensorLogTask, (void *) Sensor)) != 0)
		printf("%s : Impossible de créer Tâche Log de %s => retval = %d\n", __FUNCTION__, Sensor->Name, retval);


	pthread_attr_destroy(&attr);

	return 0;
}


int SensorsLogsInit (SensorStruct SensorTab[]) {
	int16_t	  i, numLog = 0;
	int16_t	  retval = 0;

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			numLog++;
		}
	}
	pthread_barrier_init(&LogStartBarrier, NULL, numLog+1);
	pthread_mutex_init(&Log_Mutex, NULL);

	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			printf("SensorTab[%d] =  %s\n", i, SensorTab[i].DevName);
			if ((retval = InitSensorLog(&SensorTab[i])) < 0) {
				printf("%s : Impossible d'initialiser log de %s => retval = %d\n", __FUNCTION__, SensorTab[i].Name, retval);
				return -1;
			}
		}
	}
	return 0;
};


int SensorsLogsStart (void) {
	LogActivated = 1;
	pthread_barrier_wait(&(LogStartBarrier));
	pthread_barrier_destroy(&LogStartBarrier);
	printf("%s NavLog démarré\n", __FUNCTION__);

	return 0;
};


int SensorsLogsStop (SensorStruct SensorTab[]) {
	int16_t	i;

	LogActivated = 0;
	for (i = 0; i < NUM_SENSOR; i++) {
		if (SensorTab[i].DoLog == 1) {
			pthread_join(SensorTab[i].LogThread, NULL);
			SensorTab[i].DoLog = 0;
		}
	}
	pthread_mutex_destroy(&Log_Mutex);

	return 0;
};


