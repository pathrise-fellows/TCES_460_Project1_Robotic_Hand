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
#include <math.h>

const int BASE = 100;
const int SPI_CHAN = 0;

const int max_data_size = 4096;
const int range = 25;

using std::cout;
using std::endl;
using std::cerr;

int sock , n;
struct sockaddr_in server, from;
demo::Glove_Client glove_data;
//char buffer[max_data_size] = {0};
unsigned int length;
int finger [5]= {2,3,4,5,6};
int wrist [3]= {2,3,4};
int pressure [5]; /// Integer recieved from proto

LSM9DS1 imu(IMU_MODE_I2C, 0x6b, 0x1e);

float R_DIV = 47000.000f;
const float STR_R[5]= {8500.00f,8500.00f,12000.00f,8000.00f,8000.00f};
//			 pinky ring middle  index  thumb
const float BEND_R[5] = {18000.00f,21000.00f,40000.00f,21000.00f,16000.00f};

const int PWM[8] = {25,24,23,22,21,28,29,26};
float flex_data[5];
float flex_voltage[5];
float resistance[5];
float buzzer_val[5];
int servo_val[8];

const float PI = 3.141592653589793238;
int ROLLMAX = 3.0*10.0;
int ROLLMIN = 0;
int PITCHMAX = 20;
int PITCHMIN = 0;
int YAWMIN = 39;//This depends on the orientation of the IMU, needs to be adjusted if we change location/starting orientation
int YAWMAX = 80;//This also depends on statring orientation, will need to be adjusted

void error(const char *msg){
      perror(msg);
     exit(0);
}

void hand_setup(void){
    for(int i =0; i <5; i++){
        glove_data.add_finger(i);
    }
    for(int i =0; i <3; i++ ){
        glove_data.add_wrist(i);
    }
}

void server_setup(void){
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
    for(int i =0; i<3;i++){
        printf("%d\n", glove_data.wrist(i));
    }
}

void send_data(void){
    char buffer[max_data_size] = {0};
    all_print();
    for(int i =0; i < 5; i++){
        glove_data.set_finger(i,finger[i]+1);
    }
    for(int i=0;i<3;i++){
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
        printf("%d\n", pressure[i]);
    }
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
		softPwmCreate(PWM[i],0,range);
	}
}

void servo_write(int size) {
	for (int i=0; i<size; i++) {
		softPwmWrite(PWM[i],servo_val[i]);
	}
}

void servo_val_set() {
	for (int i=0; i<5; i++) {
		servo_val[i] = pressure[i];
	}
}

void flex_read(int base) {
	for (int i=0; i<5; i++) {
		flex_data[i] = (float)analogRead(base+i);
	}
}

void calc_all(int size) {
	for (int i=0; i<size; i++) {
		flex_voltage[i] = (float)(flex_data[i]*(5.0f)/1023.0f);
		resistance[i] = (float)(R_DIV*(5.0f/flex_voltage[i] - 1.0f));
		finger[i] = map(resistance[i],STR_R[i],BEND_R[i],0,range);
	}
}

float gyro[3];
float accel[3];
float mag[3];

imu_read_calc() {
		while(!imu.gyroAvailable());
		imu.readGyro();
		while(!imu.accelAvailable());
		imu.readAccel();
		while(!imu.magAvailable());
		imu.readMag();
		printf("gyro: %f, %f, %f [deg/s]\n", imu.calcGyro(imu.gx), imu.calcGyro(imu.gy), imu.calcGyro(imu.gz));
		printf("Accel: %f, %f, %f [Gs]\n", imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az));
		printf("Mag: %f, %f, %f [gauss]\n", imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz));
		float accXnorm = imu.ax/sqrt(pow(imu.ax,2)+pow(imu.ay,2)+pow(imu.az,2));
		float accYnorm = imu.ay/sqrt(pow(imu.ax,2)+pow(imu.ay,2)+pow(imu.az,2));
		float accZnorm = imu.az/sqrt(pow(imu.ax,2)+pow(imu.ay,2)+pow(imu.ax,2));
		float pitch = asin(accXnorm);
		float outputPitch = (pitch +1.0)*10.0;
		float roll = -asin(accYnorm/cos(pitch));
		float outputRoll = (roll +1.0)*10.0;
		float magXcomp = imu.mx*cos(pitch)+imu.mz*sin(pitch);
		float magYcomp = imu.mx*sin(roll)*sin(pitch)+imu.my*cos(roll)-imu.mz*sin(roll)*cos(pitch);
		printf("pitch: %f \n", outputPitch);
		printf("roll: %f \n", roll);
		float heading = 180*atan2(magYcomp,magXcomp)/PI;
		printf("heading: %f\n", heading);
		wrist[0] = map((int)outputRoll,ROLLMIN,ROLLMAX,0,range);
		wrist[1] = map((int)outputPitch,PITCHMIN,PITCHMAX,0,range);
		wrist[2] = map((int)heading,YAWMIN,YAWMAX,0,range);
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
	imu.begin();
   	if (!imu.begin()) {
        	fprintf(stderr, "Failed to communicate with LSM9DS1.\n");
        	exit(EXIT_FAILURE);
   	}
    	imu.calibrate();
	// servo_setup(5);
	for (int i=0; i<10; i++) {
		flex_read(BASE);
		calc_all(5);
        	//all_print();
		send_data();
		receive();
		pressure_sensor_print();
		// servo_set_val();
		// servo_write(5);
		
		delay(1000);
	}
	close(sock);
    printf("client finish\n");
    
   	return 0;   
}









    


