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
  delay(100);
    int i;
    int input[7];
    String finals[5];
    int temp;
    int sequence[20];
    if(Serial.peek()==45){
      Serial.read();
      for (int i=0;i<20;i++){
         int temp2=(Serial.read());
         sequence[i]=temp2-'0';
      }

     finals[0]=String(sequence[0])+String(sequence[1])+String(sequence[2]);
     finals[1]=String(sequence[4])+String(sequence[5])+String(sequence[6]);
     finals[2]=String(sequence[8])+String(sequence[9])+String(sequence[10]);
     finals[3]=String(sequence[12])+String(sequence[13])+String(sequence[14]);
     finals[4]=String(sequence[16])+String(sequence[17])+String(sequence[18]);
     
      Serial.println(finals[0]+" "+finals[1]+" "+finals[2]+" "+finals[3]+" "+finals[4]);
      SetPosition(servoNumOne,finals[0].toInt()*angleToServ);
      SetPosition(servoNumTwo,finals[1].toInt()*angleToServ);
      
      thrustOne.write(finals[2].toInt());
      thrustTwo.write(finals[3].toInt());
      thrustThree.write(finals[4].toInt());
      }
      else{
       Serial.println(Serial.read()); 
      }

    }

