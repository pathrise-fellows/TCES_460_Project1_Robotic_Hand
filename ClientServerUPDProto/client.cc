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
const int max_data_size = 4096;

using std::cout;
using std::endl;
using std::cerr;

int sock , n;
struct sockaddr_in server, from;
demo::Hand hand_data;
std::string data;
char buffer[max_data_size];
unsigned int length;
float fArray [5]= {1.23f,2.23f,3.23f,4.23f,5.23f};
float aArray [3]= {1.23f,2.23f,3.23f};
float gArray [3]= {1.23f,2.23f,3.23f};
float mArray [3]= {1.23f,2.23f,3.23f};
float press_array [5];

void error(const char *msg){

      perror(msg);
     exit(0);
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
}
void hand_setup(void){
    for(float i =0.00f; i <5.00f; i = i + 1.00f ){
        hand_data.add_flex_resistor(i);
    }
    for(float i =0.00f; i <3.00f; i = i + 1.00f ){
        hand_data.add_imu_accel(i);
    }
    
    for(float i =0.00f; i <3.00f; i = i + 1.00f ){
        hand_data.add_imu_gyro(i);
    }
    
    for(float i =0.00f; i <3.00f; i = i + 1.00f ){
        hand_data.add_imu_mag(i);
    }
}

void fResistor_set(){
    for(int i =0; i < 5; i++){
        hand_data.set_flex_resistor(i,fArray[i]);
    }
    for(int i=0;i<3;i++){
        hand_data.set_imu_accel(i,aArray[i]);
    }
    for(int i=0;i<3;i++){
        hand_data.set_imu_gyro(i,gArray[i]);
    }
    for(int i=0;i<3;i++){
        hand_data.set_imu_mag(i,mArray[i]);
    }
}

void fResistor_print(void){
    printf("Flex resistor\n");
    for(int i =0; i <5; i++){
        printf("%f\n", hand_data.flex_resistor(i));
    }
    printf("accel\n");
    for(int i =0; i<3;i++){
        printf("%f\n", hand_data.imu_accel(i));
    }
    printf("gryo\n");
    for(int i =0; i<3;i++){
        printf("%f\n", hand_data.imu_gyro(i));
    }
    printf("mag\n");
    for(int i =0; i<3;i++){
        printf("%f\n", hand_data.imu_mag(i));
    }
}

void send_data_test(void){
    n=sendto(sock,"hello\n",
        6,0,(const struct sockaddr *)&server,length);
}

void send_data(void){
    hand_data.SerializeToString(&data);
    sprintf(buffer, "%s", data.c_str());
    n=sendto(sock,buffer,
            strlen(buffer),0,(const struct sockaddr *)&server,length);
    if (n < 0) error("Sendto");
}
void recive(){
    n = recvfrom(sock,buffer,max_data_size,0,(struct sockaddr *)&from,&length);
    if (n < 0) error("recvfrom");
    printf("receive glove_data  \n");
    std::string a = buffer;
    demo::Glove glove_data;
    glove_data.ParseFromString(a);
    printf("before for loop\n");
    for(int i =0; i < 5; i++){
        press_array[i] = glove_data.pressure_sensor(i);
    }
    printf("after for loop\n");
}
void pressure_sensor_print(void){
    printf("Flex resistor\n");
    for(int i =0; i <5; i++){
        printf("%f\n", press_array[i]);
    }
}


int main(void){
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    sever_setup();
    hand_setup();
    fResistor_set();
    send_data();
    recive();
    pressure_sensor_print();
    close(sock);
    printf("client finish\n");
    return 0;   
}









    


