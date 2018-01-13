

#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>


int pressure_data[5];
int servo_data[6];

int PWM_1 = 25;
int PWM_2 = 24;
int PWM_3 = 23;
int PWM_4 = 22;
int PWM_5 = 21;

int PWM = {25,24,23,22,21};

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



int main() {
	
	wiringPiSetup();
	/*
	Order:
	
	Read Analog
	Send Data
	Receive Data
	Calculate Data
	Write to Servo
	
	*/
	
}