
#include <Servo.h>

int pwm = 9;
int dig = 2;
Servo myservo;
uint16_t pulse0;
uint16_t min16 = 540;
uint16_t max16 = 2400;
uint8_t angle = 90;
int pos = 0;
void setup() {
  // put your setup code here, to run once:
  //pinMode(pwm,OUTPUT);
  pinMode(dig,OUTPUT);
  Serial.begin(115200);
  myservo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  //analogWrite(pwm,0);
  //digitalWrite(dig,HIGH);
  delay(1000);
  //digitalWrite(dig,LOW);
  //analogWrite(pwm,0);
  delay(1000);
  pulse0 =(min16*16L*clockCyclesPerMicrosecond() + (max16-min16)*(16L*clockCyclesPerMicrosecond())*angle/180L)/64L;
  Serial.print("clockCyclesPerMicrosecond: ");
  Serial.print(clockCyclesPerMicrosecond());
  Serial.print(", pulse0: ");
  Serial.print(pulse0);
  Serial.println();
  delay(1000);
}
