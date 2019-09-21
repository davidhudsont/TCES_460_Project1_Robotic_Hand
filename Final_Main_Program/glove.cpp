/**
 * @file glove.cpp
 * @author David Hudson, Thien Nguyen, David Vercillo
 * @brief File for testing glove sensors.
 * @version 0.6
 * @date 2019-09-21
 * 
 * 
 * 
 */
#include <iostream>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <LSM9DS1.h>
#include <LSM9DS1_Types.h>
#include <softPwm.h>
//#include <mcp3008.h>

//LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

int R_DIV = 47000;

using std::cout;

const float STR_R[5]= {8500,8500,12000,8000,8000};
//			 pinky ring middle  index  thumb
const float BEND_R[5] = {18000,21000,40000,21000,16000};
const int PWM[5] = {25,24,23,22,21};
int servo_val[5];
float resistance[5];

float angle[5];
float flex_data[5];
float flex_voltage[5];
int buzzer_val[5];
float imu_accel[3];
float imu_gyro[3];
float imu_mag[3];


#define BASE 100
#define SPI_CHAN 0
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void servo_setup(int size) {
	for (int i=0; i<size; i++) {
		pinMode(PWM[i],OUTPUT);
		softPwmCreate(PWM[i],0,50);
	}
}

void servo_write(int size) {
	
	for (int i=0; i<size; i++) {
		softPwmWrite(PWM[i],servo_val[i]);
	}
	
}

void flex_read(int base) {
	for (int i=0; i<5; i++) {
		flex_data[i] = analogRead(base+i);
	}
}

void calc_voltage(int size) {
	for (int i=0; i<size; i++) {
		flex_voltage[i] = flex_data[i]*(5.0)/1023.0;
	}
}

void calc_resistance(int size) {
	for (int i=0; i<size; i++) {
		resistance[i] = R_DIV*(5.0/flex_voltage[i] - 1.0);
	}
}

void calc_angle(int size) {
	for (int i=0; i<size; i++) {
		angle[i] = map(resistance[i],STR_R[i],BEND_R[i],0,90.0);	
	}
}

void calc_all(int size) {
	for (int i=0; i<size; i++) {
		flex_voltage[i] = flex_data[i]*(5.0)/1023.0;
		resistance[i] = R_DIV*(5.0/flex_voltage[i] - 1.0);
		angle[i] = map(resistance[i],STR_R[i],BEND_R[i],0,90.0);	
	}
}
/*
void imu_read() {
		while (!imu.gyroAvailable()) ;
        imu.readGyro();
		imu_gyro[0] = imu.calcGyro(imu.gx);
		imu_gyro[1] = imu.calcGyro(imu.gy);
		imu_gyro[2] = imu.calcGyro(imu.gz);
        while(!imu.accelAvailable()) ;
        imu.readAccel();
		imu_accel[0] = imu.calcAccel(imu.ax);
		imu_accel[1] = imu.calcAccel(imu.ay);
		imu_accel[2] = imu.calcAccel(imu.az);
        while(!imu.magAvailable()) ;
        imu.readMag();
		imu_mag[0] = imu.calcMag(imu.mx);
		imu_mag[1] = imu.calcMag(imu.my);
		imu_mag[2] = imu.calcMag(imu.mz);
}
*/
int main() {
	wiringPiSetup();
	int check;
	check = mcp3004Setup(BASE,SPI_CHAN);
	if (check == -1) {
		fprintf(stderr, "Failed to communicate with ADC_Chip.\n");
        	exit(EXIT_FAILURE);
	}
	/*
   	imu.begin();
	
   	if (!imu.begin()) {
        	fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        	exit(EXIT_FAILURE);
   	}
    imu.calibrate();
	*/
	pinMode(PWM[0],OUTPUT);
	softPwmCreate(PWM[0],0,50);
	while (1){
		flex_read(BASE);
		calc_all(5);
		/*
		for (int i =0; i<5; i++) {
			cout << "Resistance: " << resistance[i] <<" Angle : "<< angle[i] << " i: " << i << "\n";
			

		}
		*/
		cout << "Angle: " << angle[3] << "\n";
		//softPwmWrite(PWM[0],(int)angle[3]);
		softPwmWrite(PWM[0],50);
		delay(1000);
	}
	/*
	Order:
	Setup
	while(1) {
	Read Analog
	Calculate Data
	Send Data
	Receive Data
	Write to Buzzer
	}
	*/
	
    exit(EXIT_SUCCESS);

}
//
