/**
 * @file hand.cpp
 * @author David Hudson, Thien Nguyen, David Vercillo
 * @brief File for testing robotic hand sensors.
 * @version 0.6
 * @date 2019-09-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <softPwm.h>
#include <iostream>

using std::cout;

float pressure_data[5];
int servo_val[6] = {25,25,25,25,25};
int PWM[5] = {25,24,23,22,21};
int R_DIV = 3220;
float resistance[5];
float pressure [5];
float voltage[5];


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

void pressure_read(int base) {
	for (int i=0; i<5; i++) {
		pressure_data[i] = analogRead(base+i);
	}
}

void calc_voltage(int size) {
	for (int i=0; i<size; i++) {
		voltage[i] = pressure_data[i]*(5.0)/1023.0;
	}
}

void calc_resistance(int size) {
	for (int i=0; i<size; i++) {
		resistance[i] = R_DIV*(5.0/voltage[i] - 1.0);
	}
}
void calc_pressure(int size) {
	for (int i =0; i<size; i++) {
		float fsrG = 1.0/resistance[i];
		if (resistance[i] <=600) {
			pressure[i] = (fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			pressure[i] = fsrG / 0.000000642857;
		}
	}
}

void calc_all(int size) {
	for (int i =0; i<size; i++) {
		voltage[i] = pressure_data[i]*(5.0)/1023.0;
		resistance[i] = R_DIV*(5.0/voltage[i] - 1.0);
		float fsrG = 1.0/resistance[i];
		if (resistance[i] <=600) {
			pressure[i] = (fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			pressure[i] = fsrG / 0.000000642857;
		}
	}
}
int main() {
	
	wiringPiSetup();
	int check;
	check = mcp3004Setup(BASE,SPI_CHAN);
	if (check == -1) {
		fprintf(stderr, "Failed to communicate with ADC_Chip.\n");
        	exit(EXIT_FAILURE);
	}
	int MAX = 20;
	//pinMode(PWM[0],OUTPUT);
	//softPwmCreate(PWM[0],0,MAX);
	
	while(1) {
		pressure_read(BASE);
		calc_all(5);
		for (int i =0;	i<5; i++){
			cout <<"PRESSURE: " << pressure[i] << ", "; 
		}
		cout << "\n";
		delay(500);
	}
	
	/*
	Order:
	Setup
	while(1) {
	Read Analog
	Calculate Data
	Send Data
	Receive Data
	Write to Servo
	}
	*/
	
}
