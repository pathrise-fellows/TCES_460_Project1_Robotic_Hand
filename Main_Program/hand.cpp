#include <stdio.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <softPwm.h>

float pressure_data[5];
int servo_val[6];
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
		softPwmcreate(PWM[i],0,50);
	}
}

void servo_write(int size) {
	
	for (int i=0; i<size; i++) {
		softPwmwrite(PWM[i],servo_val[i])
	}
	
}

void pressure_read(int BASE) {
	for (int i=0; i<5; i++) {
		pressure_data[i] = analogRead(BASE+i);
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
