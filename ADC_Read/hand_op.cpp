
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
//#include <mcp3008.h>

float ADC_0  = 0 ;
float voltage = 0;
int check = 0;
float x = 0;
int R = 3220;
float Resistance = 0;
float pressure = 0;


int data[5];

#define BASE 100
#define SPI_CHAN 0

int main() {

	wiringPiSetup();
	check = mcp3004Setup(BASE,SPI_CHAN);
	printf("check is : %d\n",check);
	while (1){
		for (int i=0; i<5; i++) {
			x = analogRead(BASE+i);
			data[i] = x;
			printf("i : %d",i);
			printf(", x : %f",x);
			voltage = x*5.0/1023.0;
			printf(", voltage : %f\n",voltage);
			Resistance = R * (5.0 / voltage - 1.0);
			pressure = R*(5.0/(voltage)-1.0);
			float fsrG = 1.0 / Resistance; // Calculate conductance
			// Break parabolic curve down into two linear slopes:
			if (Resistance <= 600) 
				pressure = (fsrG - 0.00075) / 0.00000032639;
			else
				pressure =  fsrG / 0.000000642857;
			printf(", Resistance: %f",Resistance);
			printf(", Pressure: %f",pressure);
			printf("\n");
			delay(500);
		}
	}


}
//
