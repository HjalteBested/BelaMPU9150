/*
 * I2C_MPU9150.cpp
 *
 *  Created on: Oct 14, 2013
 *      Author: Victor Zappi
 */


#include "I2C_MPU9150.h"
#include <iostream>


// --------------------------------------------------------------------------------------- //
// ------------------------------ I2cFunctions ------------------------------------------- //
// --------------------------------------------------------------------------------------- //
uint8_t I2cFunctions::readByte(uint8_t reg){
    i2c_char_t inbuf, outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = _i2c_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = _i2c_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(inbuf);
    messages[1].buf   = &inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0){
        rt_printf("Unable to send data"); 
        return 0;
    }

    return inbuf;
}

void I2cFunctions::readBytes(uint8_t reg, uint8_t count, uint8_t * inbuf){
    i2c_char_t outbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*
     * In order to read a register, we first do a "dummy write" by writing
     * 0 bytes to the register we want to read from.  This is similar to
     * the packet in set_i2c_register, except it's 1 byte rather than 2.
     */
    outbuf = reg;
    messages[0].addr  = _i2c_address;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf   = &outbuf;

    /* The data will get returned in this structure */
    messages[1].addr  = _i2c_address;
    messages[1].flags = I2C_M_RD/* | I2C_M_NOSTART*/;
    messages[1].len   = sizeof(uint8_t)*count;
    messages[1].buf   = inbuf;

    /* Send the request to the kernel and get the result back */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2C_file, I2C_RDWR, &packets) < 0) {
        rt_printf("Unable to send data");
        return;
    }

}

void I2cFunctions::writeByte(uint8_t reg, uint8_t value){
	uint8_t buf[2] = { reg, value };

	if(write(i2C_file, buf, 2) != 2){
		std::cout << "Failed to write register " << (int)reg << " on MPU9150\n";
		return;
	}
}


// --------------------------------------------------------------------------------------- //
// ------------------------------ I2C_AK8975A -------------------------------------------- //
// --------------------------------------------------------------------------------------- //
I2C_AK8975A::I2C_AK8975A(){

}

bool I2C_AK8975A::begin(uint8_t bus, uint8_t i2caddr){
	_i2c_address = i2caddr;
	
	if(initI2C_RW(bus, i2caddr, 0) > 0) return false;
	
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	uint8_t c = readByte(WHO_AM_I_AK8975A);  // Read WHO_AM_I register for AK8975A
	rt_printf("AK8975A: I AM 0x%X, I Should Be 0x%X\n", c, 0x48);
	if(c != 0x48){
	    rt_printf("Could not connect to AK8975A: 0x%X\n",c);
		return false;
	}

	usleep(1000000); 

	rt_printf("I2C_AK8975A is online...\n");

	// Get magnetometer calibration from AK8975A ROM
	initAK8975A(magCalibration);
	rt_printf("Magnetometer calibration values:\n");
	rt_printf("X-Axis sensitivity adjustment value : %f\n", magCalibration[0]);
	rt_printf("Y-Axis sensitivity adjustment value : %f\n", magCalibration[1]);
	rt_printf("Z-Axis sensitivity adjustment value : %f\n", magCalibration[2]); 

	usleep(1000000);  

	return true;
}

void I2C_AK8975A::initAK8975A(float * destination){
	uint8_t rawData[3];  // x/y/z gyro register data stored here
	writeByte(AK8975A_CNTL, 0x00); // Power down
	usleep(10000);
	writeByte(AK8975A_CNTL, 0x0F); // Enter Fuse ROM access mode
	usleep(10000);
	readBytes(AK8975A_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
	destination[0] =  (float)(rawData[0] - 128)/256. + 1.; // Return x-axis sensitivity adjustment values
	destination[1] =  (float)(rawData[1] - 128)/256. + 1.;  
	destination[2] =  (float)(rawData[2] - 128)/256. + 1.; 
}

void I2C_AK8975A::readMagData(int16_t * destination){
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	writeByte(AK8975A_CNTL, 0x01); // toggle enable data read from magnetometer, no continuous read mode!
	usleep(10000);
	// Only accept a new magnetometer data read if the data ready bit is set and 
	// if there are no sensor overflow or data read errors
	if(readByte(AK8975A_ST1) & 0x01) { // wait for magnetometer data ready bit to be set
		readBytes(AK8975A_XOUT_L, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
        destination[0] = (((int16_t)rawData[1]) << 8) | rawData[0];
        destination[1] = (((int16_t)rawData[3]) << 8) | rawData[2];
        destination[2] = (((int16_t)rawData[5]) << 8) | rawData[4];	
	}
}

void I2C_AK8975A::update(){
	
	auto current_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> delta_t = current_time-lastUpdateTime;
	if(delta_t.count() >= minUpdateInterval){
		lastUpdateTime = current_time;
		readMagData(magCount);  // Read the x/y/z adc values
	    mRes = 10.*1229./4096.; // Conversion from 1229 microTesla full scale (4096) to 12.29 Gauss full scale
	
	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental corrections
	    mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  // get actual magnetometer value, this depends on scale being set
	    my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
	    mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];   
	    
	    // Print mag values in degree/sec
	    #ifdef PRINT_DEBUG
	    // rt_printf("Mag field: (%f,%f,%f) mG\n", mx,my,mz); 
	    #endif
	}

}

// --------------------------------------------------------------------------------------- //
// ------------------------------ I2C_MPU9150 -------------------------------------------- //
// --------------------------------------------------------------------------------------- //
I2C_MPU9150::I2C_MPU9150(){

}

bool I2C_MPU9150::begin(uint8_t bus, uint8_t i2caddr){
	_i2c_address = i2caddr;
	
	if(initI2C_RW(bus, i2caddr, 0) > 0) return false;
	
	
	// Read the WHO_AM_I register, this is a good test of communication
	uint8_t c = readByte(WHO_AM_I_MPU9150);  // Read WHO_AM_I register for MPU-9150
	rt_printf("MPU9150: I AM 0x%X, I Should Be 0x68",c);
	if(c != 0x68){
	    rt_printf("Could not connect to MPU9150: 0x%X\n",c);
		return false;
	}
	
	rt_printf("MPU9150 is online...\n");
	MPU6050SelfTest(SelfTest); // Start by performing self test and reporting values
	rt_printf("x-axis self test: acceleration trim within : %f \% of factory value\n", SelfTest[0]);
	rt_printf("y-axis self test: acceleration trim within : %f \% of factory value\n", SelfTest[1]);
	rt_printf("z-axis self test: acceleration trim within : %f \% of factory value\n", SelfTest[2]);
	rt_printf("x-axis self test: gyration trim within : %f \% of factory value\n"    , SelfTest[3]);
	rt_printf("y-axis self test: gyration trim within : %f \% of factory value\n"    , SelfTest[4]);
	rt_printf("z-axis self test: gyration trim within : %f \% of factory value\n"    , SelfTest[5]);
	if(SelfTest[0] < 1.0f && SelfTest[1] < 1.0f && SelfTest[2] < 1.0f && SelfTest[3] < 1.0f && SelfTest[4] < 1.0f && SelfTest[5] < 1.0f) {
		rt_printf("Pass Selftest!\n");
		usleep(1000);
	}
	
	calibrateMPU9150(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers  
	
	rt_printf("MPU9150 bias: x=%f, y=%f, z=%f, mg=(%f,%f,%f) o/s\n", 
		accelBias[0],
		accelBias[1], 
		accelBias[2],
		gyroBias[0],
		gyroBias[1],
		gyroBias[2]
	);

	// Init lastUpdateTime;
	lastUpdateTime = std::chrono::high_resolution_clock::now();

	// display.display();
	usleep(1000000); 
	initMPU9150(); // Inititalize and configure accelerometer and gyroscope
	
	rt_printf("MPU9150 initialized for active data mode....\n"); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

	mag.begin();

	return true;
}


//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

float I2C_MPU9150::getGres(){
	switch(Gscale){
		// Possible gyro scales (and their register bit settings) are:
		// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case GFS_250DPS:  return  250.0/32768.0;
		case GFS_500DPS:  return  500.0/32768.0;
		case GFS_1000DPS: return 1000.0/32768.0;
		case GFS_2000DPS: return 2000.0/32768.0;
	}
	return  250.0/32768.0;
}

float I2C_MPU9150::getAres(){
	switch (Ascale){
		// Possible accelerometer scales (and their register bit settings) are:
		// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11). 
		// Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
		case AFS_2G:  return  2.0/32768.0;
		case AFS_4G:  return  4.0/32768.0;
		case AFS_8G:  return  8.0/32768.0;
		case AFS_16G: return 16.0/32768.0;
	}
	return 2.0/32768.0;
}


void I2C_MPU9150::readAccelData(int16_t * destination){
	uint8_t rawData[6];  // x/y/z accel register data stored here
	readBytes(ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = (((int16_t)rawData[0]) << 8) | rawData[1];
    destination[1] = (((int16_t)rawData[2]) << 8) | rawData[3];
    destination[2] = (((int16_t)rawData[4]) << 8) | rawData[5];
}


void I2C_MPU9150::readGyroData(int16_t * destination){
	uint8_t rawData[6];  // x/y/z gyro register data stored here
	readBytes(GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
	destination[0] = (((int16_t)rawData[0]) << 8) | rawData[1];
	destination[1] = (((int16_t)rawData[2]) << 8) | rawData[3];
	destination[2] = (((int16_t)rawData[4]) << 8) | rawData[5];
}

void I2C_MPU9150::readMotion6(int16_t * destination){
	uint8_t rawData[14];  // x/y/z accel register data stored here
	readBytes(ACCEL_XOUT_H, 14, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = (((int16_t)rawData[0])  << 8) | rawData[1];
    destination[1] = (((int16_t)rawData[2])  << 8) | rawData[3];
    destination[2] = (((int16_t)rawData[4])  << 8) | rawData[5];
    destination[3] = (((int16_t)rawData[8])  << 8) | rawData[9];
    destination[4] = (((int16_t)rawData[10]) << 8) | rawData[11];
    destination[5] = (((int16_t)rawData[12]) << 8) | rawData[13];
}

int16_t I2C_MPU9150::readTempData(){
	uint8_t rawData[2];  // x/y/z gyro register data stored here
	readBytes(TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
	return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}

// Configure the motion detection control for low power accelerometer mode
void I2C_MPU9150::LowPowerAccelOnlyMPU6050()
{

// The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
// Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
// above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a 
// threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
// consideration for these threshold evaluations; otherwise, the flags would be set all the time!
  
  uint8_t c = readByte(PWR_MGMT_1);
  writeByte(PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
  writeByte(PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

  c = readByte(PWR_MGMT_2);
  writeByte(PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
  writeByte(PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running
    
  c = readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
// Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
  writeByte(ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

  c = readByte(CONFIG);
  writeByte(CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
  writeByte(CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate
    
  c = readByte(INT_ENABLE);
  writeByte(INT_ENABLE, c & ~0xFF);  // Clear all interrupts
  writeByte(INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only
  
// Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
// for at least the counter duration
  writeByte(MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
  writeByte(MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate
  
  usleep (100000);  // Add delay for accumulation of samples
  
  c = readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
  writeByte(ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance
   
  c = readByte(PWR_MGMT_2);
  writeByte(PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
  writeByte(PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])  

  c = readByte(PWR_MGMT_1);
  writeByte(PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
  writeByte(PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

}


void I2C_MPU9150::initMPU9150(){  
 // wake up device
  writeByte(PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors 
  usleep(100000); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  

 // get stable time source
  writeByte(PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  usleep(200000);
  
 // Configure Gyro and Accelerometer
 // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
 // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
 // Minimum delay time is 4.9 ms which sets the fastest rate at ~200 Hz
  writeByte(CONFIG, 0x03);  
 
 // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  writeByte(SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
 
 // Set gyroscope full scale range
 // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c =  readByte(GYRO_CONFIG);
  writeByte(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(GYRO_CONFIG, c | Gscale << 3); // Set full scale range for the gyro
   
 // Set accelerometer configuration
  c =  readByte(ACCEL_CONFIG);
  writeByte(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5] 
  writeByte(ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
  writeByte(ACCEL_CONFIG, c | Ascale << 3); // Set full scale range for the accelerometer 

 /*
 // Configure Magnetometer for FIFO
 // Initialize AK8975A for write
   writeRegister(I2C_SLV1_ADDR, 0x0C);  // Write address of AK8975A
   writeRegister(I2C_SLV1_REG, 0x0A);   // Register from within the AK8975 to which to write
   writeRegister(I2C_SLV1_DO, 0x01);    // Register that holds output data written into Slave 1 when in write mode
   writeRegister(I2C_SLV1_CTRL, 0x81);  // Enable Slave 1

 // Set up auxilliary communication with AK8975A for FIFO read
   writeRegister(I2C_SLV0_ADDR, 0x8C); // Enable and read address (0x0C) of the AK8975A
   writeRegister(I2C_SLV0_REG, 0x03);  // Register within AK8975A from which to start data read
   writeRegister(I2C_SLV0_CTRL, 0x86); // Read six bytes and swap bytes
   
 // Configure FIFO
   writeRegister(INT_ENABLE, 0x00); // Disable all interrupts
   writeRegister(FIFO_EN, 0x00);    // Disable FIFO
   writeRegister(USER_CTRL, 0x02);  // Reset I2C master and FIFO and DMP
   writeRegister(USER_CTRL, 0x00);  // Disable FIFO 
   delay(100);
// writeRegister(FIFO_EN, 0xF9); // Enable all sensors for FIFO 
// writeRegister(I2C_MST_DELAY_CTRL, 0x80); // Enable delay of external sensor data until all data registers have been read
*/ 

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
  // can join the I2C bus and all can be controlled by the Arduino as master
   writeByte(INT_PIN_CFG, 0x22);    
   writeByte(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void I2C_MPU9150::calibrateMPU9150(float * dest1, float * dest2)
{  
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
	
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	writeByte(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	usleep(100000);  
	
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	writeByte(PWR_MGMT_1, 0x01);  
	writeByte(PWR_MGMT_2, 0x00); 
	usleep(200000);
	
	// Configure device for bias calculation
	writeByte(INT_ENABLE, 0x00);   // Disable all interrupts
	writeByte(FIFO_EN, 0x00);      // Disable FIFO
	writeByte(PWR_MGMT_1, 0x00);   // Turn on internal clock source
	writeByte(I2C_MST_CTRL, 0x00); // Disable I2C master
	writeByte(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	writeByte(USER_CTRL, 0x0C);    // Reset FIFO and DMP
	usleep(15000);
	
	// Configure MPU6050 gyro and accelerometer for bias calculation
	writeByte(CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	writeByte(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	writeByte(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	writeByte(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	
	uint  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g
	
	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	writeByte(USER_CTRL, 0x40);   // Enable FIFO  
	writeByte(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
	usleep(80000); // accumulate 80 samples in 80 milliseconds = 960 bytes
	
	// At end of sample accumulation, turn off FIFO sensor read
	writeByte(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	readBytes(FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	
	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		readBytes(FIFO_R_W, 12, &data[0]); // read data for averaging
		accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
		gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
		
		accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
	
	}
	accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
	
	if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity;}
	
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1]/4)       & 0xFF;
	data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2]/4)       & 0xFF;
	
	// Push gyro biases to hardware registers
	writeByte(XG_OFFS_USRH, data[0]);
	writeByte(XG_OFFS_USRL, data[1]);
	writeByte(YG_OFFS_USRH, data[2]);
	writeByte(YG_OFFS_USRL, data[3]);
	writeByte(ZG_OFFS_USRH, data[4]);
	writeByte(ZG_OFFS_USRL, data[5]);
	
	// Output scaled gyro biases for display in the main program
	dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;  
	dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
	dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;
	
	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.
	
	int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
	readBytes(XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
	accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	readBytes(YA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	readBytes(ZA_OFFSET_H, 2, &data[0]);
	accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];
	
	uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
	
	for(ii = 0; ii < 3; ii++) {
		if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
	}
	
	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1]/8);
	accel_bias_reg[2] -= (accel_bias[2]/8);
	
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	
	// Push accelerometer biases to hardware registers
	writeByte(XA_OFFSET_H, data[0]);
	writeByte(XA_OFFSET_L_TC, data[1]);
	writeByte(YA_OFFSET_H, data[2]);
	writeByte(YA_OFFSET_L_TC, data[3]);
	writeByte(ZA_OFFSET_H, data[4]);
	writeByte(ZA_OFFSET_L_TC, data[5]);
	
	// Output scaled accelerometer biases for display in the main program
	dest2[0] = (float)accel_bias[0]/(float)accelsensitivity; 
	dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
	dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void I2C_MPU9150::MPU6050SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t rawData[4];
   uint8_t selfTest[6];
   float factoryTrim[6];
   
   // Configure the accelerometer for self-test
   writeByte(ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
   writeByte(GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   usleep(250000);  // Delay a while to let the device execute the self-test
   rawData[0] = readByte(SELF_TEST_X); // X-axis self-test results
   rawData[1] = readByte(SELF_TEST_Y); // Y-axis self-test results
   rawData[2] = readByte(SELF_TEST_Z); // Z-axis self-test results
   rawData[3] = readByte(SELF_TEST_A); // Mixed-axis self-test results
   // Extract the acceleration test results first
   selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
   selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
   selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
   // Extract the gyration test results first
   selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
   selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
   selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer   
   // Process results to allow final comparison with factory set values
   factoryTrim[0] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[0] - 1.0)/30.0))); // FT[Xa] factory trim calculation
   factoryTrim[1] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[1] - 1.0)/30.0))); // FT[Ya] factory trim calculation
   factoryTrim[2] = (4096.0*0.34)*(pow( (0.92/0.34) , (((float)selfTest[2] - 1.0)/30.0))); // FT[Za] factory trim calculation
   factoryTrim[3] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[3] - 1.0) ));             // FT[Xg] factory trim calculation
   factoryTrim[4] =  (-25.0*131.0)*(pow( 1.046 , ((float)selfTest[4] - 1.0) ));             // FT[Yg] factory trim calculation
   factoryTrim[5] =  ( 25.0*131.0)*(pow( 1.046 , ((float)selfTest[5] - 1.0) ));             // FT[Zg] factory trim calculation
   
 //  Output self-test results and factory trim calculation if desired
 //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
 //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
 //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
 //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get to percent, must multiply by 100 and subtract result from 100
   for (int i = 0; i < 6; i++) {
     destination[i] = 100.0 + 100.0*((float)selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
   }
   
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void I2C_MPU9150::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
	float norm;
	float hx, hy, _2bx, _2bz;
	float s1, s2, s3, s4;
	float qDot1, qDot2, qDot3, qDot4;
	
	// Auxiliary variables to avoid repeated arithmetic
	float _2q1mx;
	float _2q1my;
	float _2q1mz;
	float _2q2mx;
	float _4bx;
	float _4bz;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;
	float _2q4 = 2.0f * q4;
	float _2q1q3 = 2.0f * q1 * q3;
	float _2q3q4 = 2.0f * q3 * q4;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;
	
	// Normalise accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	ax *= norm;
	ay *= norm;
	az *= norm;
	
	// Normalise magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // handle NaN
	norm = 1.0f/norm;
	mx *= norm;
	my *= norm;
	mz *= norm;
	
	// Reference direction of Earth's magnetic field
	_2q1mx = 2.0f * q1 * mx;
	_2q1my = 2.0f * q1 * my;
	_2q1mz = 2.0f * q1 * mz;
	_2q2mx = 2.0f * q2 * mx;
	hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
	hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
	_2bx = sqrt(hx * hx + hy * hy);
	_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
	_4bx = 2.0f * _2bx;
	_4bz = 2.0f * _2bz;
	
	// Gradient decent algorithm corrective step
	s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
	norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
	norm = 1.0f/norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;
	s4 *= norm;
	
	// Compute rate of change of quaternion
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;
	
	// Integrate to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
	norm = 1.0f/norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;

}

 // Similar to Madgwick scheme but uses proportional and integral filtering on the error between estimated reference vectors and
 // measured ones.
void I2C_MPU9150::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz){
   float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
   float norm;
   float hx, hy, bx, bz;
   float vx, vy, vz, wx, wy, wz;
   float ex, ey, ez;
   float pa, pb, pc;

   // Auxiliary variables to avoid repeated arithmetic
   float q1q1 = q1 * q1;
   float q1q2 = q1 * q2;
   float q1q3 = q1 * q3;
   float q1q4 = q1 * q4;
   float q2q2 = q2 * q2;
   float q2q3 = q2 * q3;
   float q2q4 = q2 * q4;
   float q3q3 = q3 * q3;
   float q3q4 = q3 * q4;
   float q4q4 = q4 * q4;

   // Normalise accelerometer measurement
   norm = sqrt(ax * ax + ay * ay + az * az);
   if (norm == 0.0f) return; // handle NaN
   norm = 1.0f / norm;        // use reciprocal for division
   ax *= norm;
   ay *= norm;
   az *= norm;

   // Normalise magnetometer measurement
   norm = sqrt(mx * mx + my * my + mz * mz);
   if (norm == 0.0f) return; // handle NaN
   norm = 1.0f / norm;        // use reciprocal for division
   mx *= norm;
   my *= norm;
   mz *= norm;

   // Reference direction of Earth's magnetic field
   hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
   hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
   bx = sqrt((hx * hx) + (hy * hy));
   bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

   // Estimated direction of gravity and magnetic field
   vx = 2.0f * (q2q4 - q1q3);
   vy = 2.0f * (q1q2 + q3q4);
   vz = q1q1 - q2q2 - q3q3 + q4q4;
   wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
   wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
   wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

   // Error is cross product between estimated direction and measured direction of gravity
   ex = (ay * vz - az * vy) + (my * wz - mz * wy);
   ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
   ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
   if (Ki > 0.0f){
       eInt[0] += ex;      // accumulate integral error
       eInt[1] += ey;
       eInt[2] += ez;
   }
   else {
       eInt[0] = 0.0f;     // prevent integral wind up
       eInt[1] = 0.0f;
       eInt[2] = 0.0f;
   }

   // Apply feedback terms
   gx = gx + Kp * ex + Ki * eInt[0];
   gy = gy + Kp * ey + Ki * eInt[1];
   gz = gz + Kp * ez + Ki * eInt[2];

   // Integrate rate of change of quaternion
   pa = q2;
   pb = q3;
   pc = q4;
   q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
   q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
   q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
   q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

   // Normalise quaternion
   norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
   norm = 1.0f / norm;
   q[0] = q1 * norm;
   q[1] = q2 * norm;
   q[2] = q3 * norm;
   q[3] = q4 * norm;

}


void I2C_MPU9150::update(){  
	// If intPin goes high or data ready status is TRUE, all data registers have new data
	if(readByte(INT_STATUS) & 0x01){  // On interrupt, check if data ready interrupt
		readMotion6(motion6);  // Read the motion values
		float aRes = getAres();
		float gRes = getGres();
		// Calculate the accleration value into actual g's
		ax = (float)motion6[0]*aRes;  // get actual g value, this depends on scale being set
		ay = (float)motion6[1]*aRes;   
		az = (float)motion6[2]*aRes;    
		// Calculate the gyro value into actual degrees per second
		gx = (float)motion6[3]*gRes;  // get actual gyro value, this depends on scale being set
		gy = (float)motion6[4]*gRes;  
		gz = (float)motion6[5]*gRes;   
		

		tempCount = readTempData();  // Read the x/y/z adc values
		temperature = ((float) tempCount) / 340. + 36.53; // Temperature in degrees Centigrade
	}

	// Update Magnetometer
	mag.update();

	// Get time elapsed since last update
	auto current_time = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> delta_t = current_time-lastUpdateTime;
	deltat = delta_t.count();
	lastUpdateTime = current_time;

	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of the magnetometer;
	// the magnetometer z-axis (+ down) is opposite to z-axis (+ up) of accelerometer and gyro!
	// We have to make some allowance for this orientation mismatch in feeding the output to the quaternion filter.
	// For the MPU-9150, we have chosen a magnetic rotation that keeps the sensor forward along the x-axis just like
	// in the LSM9DS0 sensor. This rotation can be modified to allow any convenient orientation convention.
	// This is ok by aircraft orientation standards!  
	// Pass gyro rate as rad/s
	switch(quaternionUpdateType){
		case Madgwick:
		MadgwickQuaternionUpdate(ax, ay, az, gx*degToRad, gy*degToRad, gz*degToRad, mag.my, mag.mx, mag.mz);
		break;
		case Mahony:
		MahonyQuaternionUpdate  (ax, ay, az, gx*degToRad, gy*degToRad, gz*degToRad, mag.my, mag.mx, mag.mz);
		break;
	}

	// Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
	// In this coordinate system, the positive z-axis is down toward Earth. 
	// Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
	// Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
	// These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
	// Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
	// applied in the correct order which for this configuration is yaw, pitch, and then roll.
	// For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
	yaw   = atan2f(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);   
	pitch = -asinf(2.0f * (q[1] * q[3] - q[0] * q[2]));
	roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	pitch *= radToDeg;
	yaw   *= radToDeg; 
	roll  *= radToDeg;

    #ifdef PRINT_DEBUG
    // rt_printf("Acceleration: (%f,%f,%f) mg\n",ax,ay,az); 

    // Print gyro values in degree/sec
    // rt_printf("Gyro rate: (%f,%f,%f) degrees/sec\n", gx,gy,gz); 

    // Print temperature in degrees Centigrade      
    // rt_printf("Temperature is %f degrees C\n", temperature);
    rt_printf("Yaw=%f, Pitch=%f, Roll=%f, AvgRate=%f Hz\n",yaw,pitch,roll,1.0f/deltat);
    #endif 


}


