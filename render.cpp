#include <Bela.h>
#include "I2C_MPU9150.h"
#include <cmath>

// ---- internal stuff -- do not change -----

I2C_MPU9150 mpu9150; // Object to handle MPU9150 sensing


// Change this to change how often the MPU9150 is read (in Hz)
int readInterval = 200;

// ---- internal stuff -- do not change -----
AuxiliaryTask i2cTask;		// Auxiliary task to read I2C
void readMPU9150(void*);
int readCount = 0;			// How long until we read again...
int readIntervalSamples = 0; // How many samples between reads



bool setup(BelaContext *context, void *userData)
{
	if(!mpu9150.begin()){
		rt_printf("Error initialising MPU9150\n");
		return false;
	}
	
	i2cTask = Bela_createAuxiliaryTask(readMPU9150, 80, "bela-mpu9150");
	readIntervalSamples = context->audioSampleRate / readInterval;

	
	return true;
}

void render(BelaContext *context, void *userData)
{
	for(unsigned int n = 0; n < context->audioFrames; n++) {

		// Keep this code: it schedules the MPU9150 sensor readings
		if(++readCount >= readIntervalSamples) {
			readCount = 0;
			Bela_scheduleAuxiliaryTask(i2cTask);
		}


	}
}

void cleanup(BelaContext *context, void *userData)
{
	
}

// Auxiliary task to read the I2C board
void readMPU9150(void*)
{
	mpu9150.update();
}


