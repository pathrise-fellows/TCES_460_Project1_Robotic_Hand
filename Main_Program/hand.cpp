#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <softPwm.h>

float pressure_data[5];
int servo_data[6];
int PWM = {25,24,23,22,21};
int R = 3220;
int resistance[5];
float pressure [5];
float voltage[5];

#define BASE 100
#define SPI_CHAN 0



void servo_setup(int size) {
	for (int i=0; i<size; i++) {
		pinMode(PWM[i],OUTPUT);
		softPwmcreate(PWM[i],0,50);
	}
}

void servo_write(int * data) {
	
	for (int i=0; i<6; i++) {
		// Do some math
		
		softPwmwrite(PWM[i],data[i])
	}
	
}

void pressure_read(int * data.int BASE) {
	for (int i=0; i<5; i++) {
		data[i] = analogRead(BASE+i);
	}
}



void calc_voltage(int size) {
	for (int i=0; i<size; i++) {
		float voltage[i] = data[i]*(5.0)/1023.0;
	}
}

void calc_resistance(int size) {
	for (int i=0; i<size; i++) {
		float resistance[i] = R*(5.0)/voltage[i] - 1.0);
	}
}
void calc_pressure(int size) {
	float fsrG = 1.0/Res;
	for (int i =0; i<size; i++) {
		if (resistance[i] <=600) {
			pressure[i] = fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			pressure[i] = fsrG / 0.000000642857;
		}
	}
}

void calc(int size) {
	for (int i =0; i<size; i++) {
		resistance[i] = R*(5.0)/(data[i]*(5.0)/1023.0) - 1.0);
		float fsrG = 1.0/resistance[i];
		if (resistance[i] <=600) {
			pressure[i] = fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			pressure[i] = fsrG / 0.000000642857;
		}
	}
}
int main() {
	
	wiringPiSetup();
	mcp3004Setup(BASE,SPI_CHAN);
	
	/*
	Order:
	Setup
	while(1) {
	Read Analog
	Send Data
	Receive Data
	Calculate Data
	Write to Servo
	}
	*/
	
}