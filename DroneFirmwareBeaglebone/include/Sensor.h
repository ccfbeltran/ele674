/*
 * Sensor.h
 *
 *  Created on: 19 déc. 2012
 *      Author: bruno
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#define _POSIX_C_SOURCE 200809L
#define _XOPEN_SOURCE 500

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <poll.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>
#include <math.h>

#define POLICY SCHED_RR
#define THREADSTACK  65536

#define INVALID_CHECKSUM -1
#define OLD_SAMPLE        0
#define NEW_SAMPLE        1

#define NUM_SENSOR 		5
#define DATABUFSIZE 	100
#define GYROCALMAX		1000
#define SONARCALMAX		100

enum { ACCELEROMETRE, GYROSCOPE, MAGNETOMETRE, BAROMETRE, SONAR, NUM_SENSORS };

#define ACCEL_RATE		1
#define GYRO_RATE		1
#define SONAR_RATE		8
#define BAROM_RATE		2
#define MAGNETO_RATE	2

#define ACCEL_RAW_TO_ms2	(9.80665*16.0/32768.0)
#define ACCEL_RAW_TO_unity	(1.0/32768.0)
#define GYRO_RAW_TO_DegCs	(2000.0/32768.0)
#define GYRO_RAW_TO_Rads	(2000.0*M_PI/180.0/32768.0)
#define MAG_RAW_TO_uT		(4912.0/32768.0)
#define MAG_RAW_TO_unity	(1.0/32768.0)
#define PRESS_RAW_TO_Pa		(0.16)
#define TEMP_RAW_TO_DegC	(0.0025)
#define TEMP_SENSITIVITY	(333.87)
#define SONAR_uS_TO_M		(0.000172215)


#define T1	((double)((uint16_t) 0x6D4E))	// ( 27982.0)
#define T2	((double)( (int16_t) 0x67FE))	// ( 26622.0)
#define T3	((double)( (int16_t) 0xFC18))	// ( -1000.0)

#define P1	((double)((uint16_t) 0x9506))	// ( 38150.0)
#define P2	((double)( (int16_t) 0xD69F))	// (-10593.0)
#define P3	((double)( (int16_t) 0x0BD0))	// (  3024.0)
#define P4	((double)( (int16_t) 0x088D))	// (  2189.0)
#define P5	((double)( (int16_t) 0x008D))	// (   141.0)
#define P6	((double)( (int16_t) 0xFFF9))	// (    -7.0)
#define P7	((double)( (int16_t) 0x3C8C))	// ( 15500.0)
#define P8	((double)( (int16_t) 0xC6F8))	// (-14600.0)
#define P9	((double)( (int16_t) 0x1770))	// (  6000.0)


typedef struct SensorRawData_struct {
	int8_t	    	  	  type;
	int8_t	    		  status;
	uint64_t			  timestamp;
	uint32_t	    	  ech_num;
	int32_t	    	  	  data[3];
	int32_t	    	  	  temp;
} SensorRawData;


typedef struct SensorData_struct {
	uint32_t	TimeDelay;
	double		Data[3];
} SensorData;


typedef struct SensorParam_struct {
	int16_t		AbsErrorMax;
	int16_t		RelErrorMax;
	double 		minVal, centerVal, maxVal, Conversion;
	double		alpha[3][3], beta[3];
} SensorParam;


typedef struct Sensor_struct {
	const char 			*DevName;
	const char 			*Name;
	int					File;
	uint16_t			type;
	pthread_spinlock_t 	DataLock;
	pthread_mutex_t 	DataSampleMutex;
	pthread_cond_t  	DataNewSampleCondVar;
	pthread_t 			SensorThread;
	pthread_t 			LogThread;
	uint16_t			DoLog;
	uint16_t			DataIdx;
	SensorParam			*Param;
	SensorRawData		*RawData;
	SensorData			*Data;
} SensorStruct;



#define ACCEL_PARAM { .AbsErrorMax 	        = 25, \
		 	 	 	  .RelErrorMax   		= 25, \
		 	 	 	  .minVal 		 		= -32768.0, \
		 	 	 	  .centerVal 	 		= 0.0, \
		 	 	 	  .maxVal 		 		= 32767.0, \
					  .Conversion 	 		= ACCEL_RAW_TO_unity, \
					  .alpha 		 		= {{15.876175633023, -1.637420968457,  0.054287183058},{-1.637418877037, 15.937675301119,  0.074703358803},{0.054296331833,  0.074714143548, 15.896564676754}}, \
					  .beta 		 		= {0.004432402777, -0.012066240710, -0.038626717573} \
					}

#define GYRO_PARAM { .AbsErrorMax		   = 25, \
	 	 	 	 	 .RelErrorMax		   = 25, \
	 	 	 	 	 .minVal			   = -32768.0, \
	 	 	 	 	 .centerVal			   = 0.0, \
	 	 	 	 	 .maxVal			   = 32767.0, \
	 	 	 	 	 .Conversion		   = GYRO_RAW_TO_Rads, \
	 	 	 	 	 .alpha				   = {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
	 	 	 	 	 .beta				   = {0.0, 0.0, 0.0} \
				   }

#define SONAR_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 0.00034454, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

#define BAROM_PARAM { .AbsErrorMax			= 25, \
					  .RelErrorMax			= 25, \
					  .minVal				= 0.0, \
					  .centerVal			= 0.0, \
					  .maxVal				= 0.0, \
					  .Conversion			= 1.0, \
					  .alpha				= {{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}, \
					  .beta					= {0.0, 0.0, 0.0} \
					}

#define MAGNETO_PARAM { .AbsErrorMax  		  = 25, \
						.RelErrorMax  		  = 25, \
		 	 	 	 	.minVal				  = -32768.0, \
		 	 	 	 	.centerVal			  = 0.0, \
		 	 	 	 	.maxVal				  = 32767.0, \
						.Conversion	  		  = MAG_RAW_TO_unity, \
						.alpha		  		  = {{127.919370506070,  6.985262859337, -2.895689707825},{7.864821471385, 126.063766067528, -2.551603930305},{-1.574317029998, -1.255080485803, 119.884157236363}}, \
						.beta		  		  = {-0.398129623475, -0.176674060132, 0.942526257273} \
					  }



#define ACCEL_INIT { .DevName 	= "/dev/i2c_accel", \
					 .Name 		= "ACCELEROMETRE (m/s^2)", \
					 .File 		= -1, \
					 .type 		= ACCELEROMETRE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[ACCELEROMETRE], \
					 .RawData	= &(RawData[ACCELEROMETRE][0]), \
					 .Data		= &(NavData[ACCELEROMETRE][0]) \
				   }

#define GYRO_INIT  { .DevName 	= "/dev/i2c_gyro", \
					 .Name 		= "GYROSCOPE", \
					 .File 		= -1, \
					 .type 		= GYROSCOPE, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[GYROSCOPE], \
					 .RawData	= &(RawData[GYROSCOPE][0]), \
					 .Data		= &(NavData[GYROSCOPE][0]) \
				   }

#define SONAR_INIT { .DevName 	= "/dev/i2c_sonar", \
					 .Name 		= "SONAR", \
					 .File 		= -1, \
					 .type 		= SONAR, \
					 .DoLog     = 0, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[SONAR], \
					 .RawData	= &(RawData[SONAR][0]), \
					 .Data		= &(NavData[SONAR][0]) \
				   }

#define BAROM_INIT { .DevName 	= "/dev/i2c_barom", \
					 .Name 		= "BAROMETRE", \
					 .File 		= -1, \
					 .type 		= BAROMETRE, \
					 .DataIdx   = 0, \
					 .Param		= &ParamData[BAROMETRE], \
					 .RawData	= &(RawData[BAROMETRE][0]), \
					 .Data		= &(NavData[BAROMETRE][0]) \
				   }

#define MAGNETO_INIT { .DevName  = "/dev/i2c_magneto", \
					   .Name 	 = "MAGNETOMETRE", \
					   .File 	 = -1, \
					   .type 	 = MAGNETOMETRE, \
					   .DoLog    = 0, \
					   .DataIdx  = 0, \
					   .Param	 = &ParamData[MAGNETOMETRE], \
					   .RawData	 = &(RawData[MAGNETOMETRE][0]), \
					   .Data	 = &(NavData[MAGNETOMETRE][0]) \
				     }


void *SensorTask(void *ptr);
void *SensorLogTask(void *ptr);

int   SensorsInit(SensorStruct SensorTab[]);
int   SensorsStart (void);
int   SensorsStop (SensorStruct SensorTab[]);
int   SensorsLogsInit (SensorStruct SensorTab[]);
int   SensorsLogsStart (void);
int   SensorsLogsStop (SensorStruct SensorTab[]);

#endif /* SENSOR_H_ */
