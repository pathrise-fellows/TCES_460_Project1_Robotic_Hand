/* Server Program to handle TCP connections */

#include <sys/types.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <stdio.h>
#include <iostream>

#include "Hand.pb.h"

#include <time.h>
#include <sys/time.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <softPwm.h>

const int max_data_size = 4096;

using std::cout;
using std::endl;
using std::cerr;

demo::Glove glove_data;


struct sockaddr_in server;
struct sockaddr_in client;
int sock,n, length;
socklen_t fromlen;
//char buf[max_data_size];
float flex_array[5];
float accel_array [3];
float gyro_array [3];
float mag_array [3];

float press_array [5] = {0.14,1.14,2.14,3.14,5555.14};

const int BASE = 100;
const int SPI_CHAN = 0;


float pressure_data[5];
int servo_val[6];
int PWM[5] = {25,24,23,22,21};
int R_DIV = 3220;
float resistance[5];
float voltage[5];

const float max_pressure[5] = {8000,8000,10000,8000,11000};
const float min_pressure[5] = {0,0,0,0,0};


void error(const char *msg){

    perror(msg);
    exit(0);
}

void server_setup(){
    int port = 1024;
    sock=socket(AF_INET, SOCK_DGRAM, 0);
       if (sock < 0) error("Opening socket");
    length = sizeof(server);
    bzero(&server,length);
    server.sin_family=AF_INET;
    server.sin_addr.s_addr=INADDR_ANY;
    server.sin_port=htons(port);
    if (bind(sock,(struct sockaddr *)&server,length)<0) 
       error("binding");
    fromlen = sizeof(struct sockaddr_in);
}
void glove_setup(){
    for(float i =0.00f; i <5.00f; i = i + 1.00f ){
        glove_data.add_pressure_sensor(i);
    }
}

void receive(){
    char buffer[max_data_size] = {0};
    n = recvfrom(sock,buffer,max_data_size,0,(struct sockaddr *)&client,&fromlen);
    if (n < 0) error("recvfrom");
    printf("receive hand_data  \n");
    std::string a = buffer;
    demo::Hand hand_data;
    hand_data.ParseFromString(a);
    for(int i =0; i < 5; i++){
        flex_array[i] = hand_data.flex_resistor(i);
    }
    for(int i =0; i < 3; i++){
        accel_array[i] = hand_data.imu_accel(i);
    }
    for(int i =0; i < 3; i++){
        gyro_array[i] = hand_data.imu_gyro(i);
    }
    for(int i =0; i < 3; i++){
        mag_array[i] = hand_data.imu_mag(i);
    }
}
void send_data(){
    char buffer[max_data_size] = {0};
    for(int i =0; i <5; i++ ){
        glove_data.set_pressure_sensor(i, press_array[i]+0.01f);
        printf("%d = %f\n",i, glove_data.pressure_sensor(i));
    }
    std::string data;
    glove_data.SerializeToString(&data);
    sprintf(buffer, "%s", data.c_str());
    n=sendto(sock,buffer,
            max_data_size,0,(const struct sockaddr *)&client,fromlen);
    if (n < 0) error("Sendto");
}

void print_in(){
    printf("Flex values\n");
    for(int i =0; i < 5; i++){
        printf("%d = %f\n",i, flex_array[i]);
    }
    printf("accel values\n");
    for(int i =0; i < 3; i++){
        printf("%d = %f\n",i, accel_array[i]);
    }
    printf("gyro value\n");
    for(int i =0; i < 3; i++){
        printf("%d = %f\n",i, gyro_array[i]);
    }
    printf("mag value\n");
    for(int i =0; i < 3; i++){
        printf("%d = %f\n",i, mag_array[i]);
    }
}

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
			press_array[i] = (fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			press_array[i] = fsrG / 0.000000642857;
		}
	}
}

void calc_all(int size) {
	for (int i =0; i<size; i++) {
		voltage[i] = pressure_data[i]*(5.0)/1023.0;
		resistance[i] = R_DIV*(5.0/voltage[i] - 1.0);
		float fsrG = 1.0/resistance[i];
		if (resistance[i] <=600) {
			press_array[i] = (fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			press_array[i] = fsrG / 0.000000642857;
		}
	}
}
int main(void){
   	GOOGLE_PROTOBUF_VERIFY_VERSION;
   	server_setup();
    glove_setup();
	wiringPiSetup();
   	int check;
    	check = mcp3004Setup(BASE,SPI_CHAN);
    	if (check == -1) {
		fprintf(stderr, "Failed to communicate with ADC_Chip.\n");
		exit(EXIT_FAILURE);
   	}
	
	for(int i =0; i <10; i++){
		receive();
		//print_in();
		pressure_read(BASE);
		calc_all(5);
		for (int i=0; i<5; i++) {
            printf("%d = %f\n",i, press_array[i]);
		}
		send_data();
		
		delay(1000);
	}
    close(sock);
    /*server_setup();
    glove_setup();
    receive();
    glove_set();
    print_in();
    send_data();
    close(sock);*/
    printf("done with server\n");
	return 0;
}
