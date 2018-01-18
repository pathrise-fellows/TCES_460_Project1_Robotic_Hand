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


const int max_data_size = 4096;

using std::cout;
using std::endl;
using std::cerr;

demo::Glove glove_data;
std::string data;


struct sockaddr_in server;
struct sockaddr_in client;
int sock, length, n;
socklen_t fromlen;
char buf[max_data_size];
float flex_array[5];
float accel_array [3];
float gyro_array [3];
float mag_array [3];

float press_array [5] = {0.14,1.14,2.14,3.14,4.14};






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

void glove_set(){
    for(int i =0; i <5; i++ ){
        glove_data.set_pressure_sensor(i, press_array[i]);
    }
}

void recive_data_test(){
    n = recvfrom(sock,buf,max_data_size,0,(struct sockaddr *)&client,&fromlen);
    if (n < 0) error("recvfrom");
    write(1,buf,n);
}
void receive(){
    n = recvfrom(sock,buf,max_data_size,0,(struct sockaddr *)&client,&fromlen);
    if (n < 0) error("recvfrom");
    printf("receive hand_data  \n");
    std::string a = buf;
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
    glove_data.SerializeToString(&data);
    sprintf(buf, "%s", data.c_str());
    n=sendto(sock,buf,
            strlen(buf),0,(const struct sockaddr *)&client,fromlen);
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


int main(void){
   	GOOGLE_PROTOBUF_VERIFY_VERSION;
   	server_setup();
    	glove_setup();
    	glove_set();
	wiringPiSetup();
   	
	
	for (int i=0; i<100; i++) {
		send_data();
		print_in();
		receive();
		delay(1000);
	}
	send_data();
    	print_in();
	receive();
    	close(sock);
    	printf("done with server\n");
	exit(EXIT_SUCCESS);
    return 0;
}
