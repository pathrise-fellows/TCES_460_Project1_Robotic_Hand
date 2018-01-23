#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "Hand.pb.h"

#include <errno.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <LSM9DS1.h>
#include <LSM9DS1_Types.h>
#include <softPwm.h>

const int BASE = 100;
const int SPI_CHAN = 0;

const int max_data_size = 4096;

using std::cout;
using std::endl;
using std::cerr;

int sock , n;
struct sockaddr_in server, from;
demo::Glove_Client glove_data;
//char buffer[max_data_size] = {0};
unsigned int length;
int finger [5]= {2,3,4,5,6};
int wrist [2]= {2,3};
int pressure [5];

//LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

float R_DIV = 47000.000f;
const float STR_R[5]= {8500.00f,8500.00f,12000.00f,8000.00f,8000.00f};
//			 pinky ring middle  index  thumb
const float BEND_R[5] = {18000.00f,21000.00f,40000.00f,21000.00f,16000.00f};

const int PWM[5] = {25,24,23,22,21};
float flex_data[5];
float flex_voltage[5];
float resistance[5];
float buzzer_val[5];
int servo_val[5];


void error(const char *msg){
      perror(msg);
     exit(0);
}

void hand_setup(void){
    for(int i =0; i <5; i++){
        glove_data.add_finger(i);
    }
    for(int i =0; i <2; i++ ){
        glove_data.add_wrist(i);
    }
}

void sever_setup(void){
    int port = 1024;
    struct hostent *hp;
    length=sizeof(struct sockaddr_in);
    sock= socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) error("socket");

    server.sin_family = AF_INET;
    hp = gethostbyname("10.16.4.131");
    bcopy((char *)hp->h_addr, 
            (char *)&server.sin_addr,
            hp->h_length);
    server.sin_port = htons(port);
    hand_setup();
}

void all_print(void){
    printf("Flex resistor\n");
    for(int i =0; i <5; i++){
        printf("%d\n", glove_data.finger(i));
    }
    printf("wrist\n");
    for(int i =0; i<2;i++){
        printf("%d\n", glove_data.wrist(i));
    }
}

void send_data(void){
    char buffer[max_data_size] = {0};
    all_print();
    for(int i =0; i < 5; i++){
        printf("%d \n", finger[i]);
        glove_data.set_finger(i,finger[i]+1);
    }
    for(int i=0;i<2;i++){
        glove_data.set_wrist(i,wrist[i]+1);
    }
    all_print();
    std::string data;
    glove_data.SerializeToString(&data);
    sprintf(buffer, "%s", data.c_str());
    n=sendto(sock,buffer,
            strlen(buffer),0,(const struct sockaddr *)&server,length);
    if (n < 0) error("Sendto");
}
void receive(){
    char buffer[max_data_size] = {0};
    n = recvfrom(sock,buffer,max_data_size,0,(struct sockaddr *)&server,&length);
    if (n < 0) error("recvfrom");
    printf("receive glove_data  \n");
    std::string a = buffer;
    demo::Hand_Server hand_data;
    hand_data.ParseFromString(a);
    printf("before for loop\n");
    for(int i =0; i < 5; i++){
        printf("%d = %d\n",i, pressure[i]);
        pressure[i] = hand_data.pressure(i)-1;
        printf("%d = %d\n",i, pressure[i]);
    }
    printf("after for loop\n");
}
void pressure_sensor_print(void){
    printf("Pressure resistor\n");
    for(int i =0; i <5; i++){
        printf("%f\n", pressure[i]);
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

void flex_read(int base) {
	for (int i=0; i<5; i++) {
		flex_data[i] = (float)analogRead(base+i);
	}
}

void calc_voltage(int size) {
	for (int i=0; i<size; i++) {
		flex_voltage[i] = (float)((flex_data[i]*(5.0f)/1023.0f));
	}
}

void calc_resistance(int size) {
	for (int i=0; i<size; i++) {
		resistance[i] = R_DIV*(5.0f/flex_voltage[i] - 1.0f);
	}
}

void calc_angle(int size) {
	for (int i=0; i<size; i++) {
		finger[i] = map(resistance[i],STR_R[i],BEND_R[i],0,90.0f);	
	}
}

void calc_all(int size) {
	for (int i=0; i<size; i++) {
		flex_voltage[i] = (float)(flex_data[i]*(5.0f)/1023.0f)+ 0.01f;
		resistance[i] = (float)(R_DIV*(5.0f/flex_voltage[i] - 1.0f))+ 0.01f;
		finger[i] = (float)(map(resistance[i],STR_R[i],BEND_R[i],0,90.0f)+ 0.01f);	
	}
}
/*
void imu_read() {
		while (!imu.gyroAvailable()) ;
        imu.readGyro();
		gArray[0] = imu.calcGyro(imu.gx);
		gArray[1] = imu.calcGyro(imu.gy);
		gArray[2] = imu.calcGyro(imu.gz);
        while(!imu.accelAvailable()) ;
        imu.readAccel();
		wrist[0] = imu.calcAccel(imu.ax);
		wrist[1] = imu.calcAccel(imu.ay);
		wrist[2] = imu.calcAccel(imu.az);
        while(!imu.magAvailable()) ;
        imu.readMag();
		mArray[0] = imu.calcMag(imu.mx);
		mArray[1] = imu.calcMag(imu.my);
		mArray[2] = imu.calcMag(imu.mz);
}
*/

int main(void){
	GOOGLE_PROTOBUF_VERIFY_VERSION;
	sever_setup();
	wiringPiSetup();
	int check;
	check = mcp3004Setup(BASE,SPI_CHAN);
	if (check == -1) {
		fprintf(stderr, "Failed to communicate with ADC_Chip.\n");
        	exit(EXIT_FAILURE);
	}
	for (int i=0; i<10; i++) {
		//flex_read(BASE);
		//calc_all(5);
        //all_print();
		send_data();
		
		receive();
		pressure_sensor_print();
		
		
		delay(1000);
	}
	close(sock);
	printf("client finish\n");
    /*
    sever_setup();
    hand_setup();
    fResistor_set();
    send_data();
    receive();
    pressure_sensor_print();
    close(sock);*/
    printf("client finish\n");
    
   	return 0;   
}









    


