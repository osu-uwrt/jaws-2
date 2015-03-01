#include <ax12.h>
#include <Servo.h>

double angleToServ = 4096/360;

Servo thrustOne;
Servo thrustTwo;
Servo thrustThree;

int servoNumOne=18;//forgot which ones i set them as
int servoNumTwo=15;//this number sounds familiar

void setup(){
Serial.begin(38400);
//thrustOne.attach(1);//arbitrary pin#
//thrustTwo.attach(2);//arbitrary pin#
//thrustThree.attach(3);//arbitrary pin#
}
void loop(){
  //delay(100);
  int i;
  int input[7];
  int finals[5];
  int temp;
  int sequence[10];
  if(Serial.read()==45){
    for (int i=0;i<10;i++){
      sequence[i]=(Serial.read());
    }
    finals[0]=256*sequence[1]+sequence[0];
    finals[1]=256*sequence[3]+sequence[2];
    finals[2]=256*sequence[5]+sequence[4];
    finals[3]=256*sequence[7]+sequence[6];
    finals[4]=256*sequence[9]+sequence[8];
    SetPosition(servoNumOne,finals[0]*angleToServ);
    SetPosition(servoNumTwo,finals[1]*angleToServ);
    Serial.println(finals[0]+" "+finals[1]);
    thrustOne.write(finals[2]);
    thrustTwo.write(finals[3]);
    thrustThree.write(finals[4]);
  }
  else{
    Serial.read();
  }
}
