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

//this is new
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <cstdlib>
///

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

demo::Hand_Server hand_data;


struct sockaddr_in server;
struct sockaddr_in client;
int sock,n, length;
socklen_t fromlen;
char buffer[max_data_size];

int finger[5];
int wrist [3];
int pressure [5] = {3,4,5,6,7};

float pressure_calc[5];
const int BASE = 100;
const int SPI_CHAN = 0;

const int range = 25;
float pressure_data[5];
int servo_val[8];
int PWM[8] = {25,24,23,22,21,28,29,26};
int R_DIV = 3220;
float resistance[5];
float voltage[5];

const float max_pressure[5] = {8000,8000,10000,8000,11000};
const float min_pressure[5] = {0,0,0,0,0};


void error(const char *msg){
    perror(msg);
    exit(0);
}

void glove_setup(){
    for(int i =0; i <5; i++ ){
        hand_data.add_pressure(i);
    }
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
    glove_setup();
}

int timeout_recvfrom(int timeoutinseconds){
    fd_set socks;
    struct timeval t;
    FD_ZERO(&socks);
    FD_SET(sock, &socks);
    t.tv_sec = timeoutinseconds;
    if (select(sock + 1, &socks, NULL, NULL, &t)){
        recvfrom(sock,buffer,max_data_size,0,(struct sockaddr *)&client,&fromlen);
        return TRUE;
    }
    else
        return FALSE;
}

void receive(){
    memset(buffer, 0, sizeof(buffer));
    n = timeout_recvfrom(5);
    //n = recvfrom(sock,buffer,max_data_size,0,(struct sockaddr *)&client,&fromlen);
    if (n < 0){
        printf("Did not receive");
        exit(EXIT_FAILURE);
    }
    printf("receive Glove_Client  \n");
    std::string a = buffer;
    demo::Glove_Client glove_data;
    glove_data.ParseFromString(a);
    for(int i =0; i < 5; i++){
        finger[i] = glove_data.finger(i)-1;
    }
    for(int i =0; i < 3; i++){
        wrist[i] = glove_data.wrist(i)-1;
    }
}
void send_data(){
    memset(buffer, 0, sizeof(buffer));
    for(int i =0; i <5; i++ ){
        hand_data.set_pressure(i, pressure[i]+1);
    }
    std::string data;
    hand_data.SerializeToString(&data);
    sprintf(buffer, "%s", data.c_str());
    n=sendto(sock,buffer,
            max_data_size,0,(const struct sockaddr *)&client,fromlen);
    if (n < 0) error("Sendto");
}

void print_recieve(){
	cout << "Start of Finger and Wrist Data Receive" << endl;
    for(int i =0; i < 5; i++){
        printf("%d = %d\n",i, finger[i]);
    }
    for(int i =0; i < 3; i++){
        printf("%d = %d\n",i, wrist[i]);
    }
	cout << "End of Finger and Wrist Data Receive" << endl;
}

void print_servo() {
	cout << "Servo Values" << endl;
	for (int i=0; i<8; i++) {
		cout << servo_val[i] << endl;
	}
	cout << "End of Servo values" << endl;
}

void print_send() {
	cout << "Start of Send Pressure values" << endl;
	for (int i =0; i<5; i++) {
		cout << pressure[i] << endl;
	}
	cout << "End of Send Pressure values" << endl;
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
	int ret =  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	if (ret > out_max) {
		ret = out_max-1;
	}
	if (ret < out_min) {
		ret = out_min+1;
	}
	return ret;
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

void servo_val_set() {
	for (int i =0; i<5; i++) {
		servo_val[i] = finger[5];
	}
	int c= 6;
	for (int i =0; i<3; i++){
		servo_val[c] = wrist[i];
		c++;
	}
}

void pressure_read(int base) {
	for (int i=0; i<5; i++) {
		pressure_data[i] = analogRead(base+i);
	}
}

void calc_all(int size) {
	for (int i =0; i<size; i++) {
		voltage[i] = pressure_data[i]*(5.0)/1023.0;
		resistance[i] = R_DIV*(5.0/voltage[i] - 1.0);
		float fsrG = 1.0/resistance[i];
		if (resistance[i] <=600) {
			pressure_calc[i] = (fsrG - 0.00075)/ 0.00000032639;
		}
		else {
			pressure_calc[i] = fsrG / 0.000000642857;
		}
        
		pressure[i] = map(pressure_calc[i], min_pressure[i], max_pressure[i], 0, range);
	}
}
int main(void){
   	GOOGLE_PROTOBUF_VERIFY_VERSION;
   	server_setup();
	wiringPiSetup();
   	int check;
    	check = mcp3004Setup(BASE,SPI_CHAN);
    	if (check == -1) {
		fprintf(stderr, "Failed to communicate with ADC_Chip.\n");
		exit(EXIT_FAILURE);
   	}
	// servo_setup(8);
	for(int i =0; i <10; i++){
		receive();
		print_recieve();
		servo_val_set();
		print_servo();
		// servo_write(8);
		pressure_read(BASE);
		calc_all(5);
		print_send();
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
