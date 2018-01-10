
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <softPwm.h>

float PWM_0  = 25;

int main() {

	wiringPiSetup();
	
	pinMode(PWM_0,OUTPUT);
	softPwmCreate(PWM_0,0,50);
	
	while (1){
			softPwmWrite(PWM_0,25);
	}


}
//
//

