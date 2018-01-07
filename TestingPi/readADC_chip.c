#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

float voltage = 0;
int channel = 0;
int speed = 2000000;
int R = 3220;
float Resistance = 0;
float pressure = 0;
int check = 0;


#define BASE 100
#define SPI_CHAN 0

int main() {

	//wiringPiSetup();
	check = wiringPiSPISetup(channel,speed);
	printf("check is : %d\n",check);

	while (1){
		unsigned char data[3] = {1};
		check = wiringPiSPIDataRW(channel,data,4);
		int x = ( (data[1] & 3 ) << 8 ) + data[3];
		voltage = x*5.0/1023.0;
		printf("Analog : %d", x);
		printf(", Voltage : %f", voltage);
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
		delay(1000);
	}
}
// Hello!
