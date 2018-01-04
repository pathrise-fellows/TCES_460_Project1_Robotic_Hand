
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define size 5

int fingers[size];
int rbtfingers[size];
char msg[256]="";
char buf[256]="";


void encode(){
    for (int i=0; i<size; i++) {
        snprintf(buf,sizeof(buf),"%d",fingers[i]);
        strcat(msg,buf);
        strcat(msg,"R");
    }
}
void decode() {
    int count = 0;
    snprintf(buf,sizeof(buf),"%s","");
    char *pch;
    pch = strtok(msg,"RE");
    while (pch !=NULL) {
        rbtfingers[count] = atoi(pch);
        count++;
        pch = strtok(NULL,"R");
    }
}
int main()
{
    fingers[0] = 10;
    fingers[1] = 20;
    fingers[2] = 30;
    fingers[3] = 40;
    fingers[4] = 50;
    encode();
	printf("%s\n",msg);
    decode();
    for (int i=0; i<size; i++) {
		printf("%d\n",rbtfingers[i]);
    }

    return 0;
}
