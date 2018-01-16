//glove file
//

#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include "LSM9DS1.h"
#include "LSM9DS1_Types.h"
//#include <mcp3008.h>

float ADC_0  = 0 ;
float voltage = 0;
int check = 0;
float x = 0;
int R = 47000;
float Resistance = 0;
float pressure = 0;

const float STRAIGHT_RESISTANCE = 14038.0; // resistance when straight
const float BEND_RESISTANCE = 68000.0; // resistance at 90 deg

const float STR_R = {14038,8000,8000,8000,8000};
const float BEND_R = {68000,40000,40000,40000,40000};
float ANGLE = {0,0,0,0,0};

int data[5];

#define BASE 100
#define SPI_CHAN 0
long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



int main() {

	wiringPiSetup();
	check = mcp3004Setup(BASE,SPI_CHAN);
	printf("check is : %d\n",check);
	
	LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);
   	imu.begin();
	
   	if (!imu.begin()) {
        	fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        	exit(EXIT_FAILURE);
   	}
    	imu.calibrate();
	while (1){
		for (int i=0; i<1; i++) {
			x = analogRead(BASE+i);
			data[i] = x;
			printf("i : %d",i);
			printf(", x : %f",x);
			voltage = x*5.0/1023.0;
			printf(", voltage : %f",voltage);
			Resistance = R * (5.0 / voltage - 1.0);
			printf(", Resistance: %f",Resistance);
			float angle = map(Resistance,STRAIGHT_RESISTANCE,BEND_RESISTANCE,0,90.0);
			printf(", Angle: %f",angle);
			printf("\n");
			delay(500);
		}
	}
    for (;;) {
        while (!imu.gyroAvailable()) ;
        imu.readGyro();
        while(!imu.accelAvailable()) ;
        imu.readAccel();
        while(!imu.magAvailable()) ;
        imu.readMag();

        printf("Gyro: %f, %f, %f [deg/s]\n", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
        printf("Accel: %f, %f, %f [Gs]\n", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
        printf("Mag: %f, %f, %f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
        sleep(1.0);
    }

    exit(EXIT_SUCCESS);

}
//
