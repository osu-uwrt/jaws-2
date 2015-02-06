   #include <ax12.h>
   #include <Servo.h>
   
   Servo thrustOne;
   Servo thrustTwo;
   Servo thrustThree;
   int servoNumOne=18;//forgot which ones i set them as
   int servoNumTwo=19;//this number sounds familiar
void setup(){
   Serial.begin(38400); 

    //thrustOne.attach(1);//arbitrary pin#
    //thrustTwo.attach(2);//arbitrary pin#
    //thrustThree.attach(3);//arbitrary pin# 
}
void loop(){
    int i;
    int input[7];
    String finals[5];
    int temp;
    delay(500);
    int sequence[15];
    Serial.print(Serial.available()+" ");
    if(Serial.peek()==45){
      Serial.read();
      while(1){
        if(Serial.available()==20){
           Serial.print("In the if");
           for (int i=0;i<20;i++){
             int temp2=(Serial.read()-'/');
             if(temp!=42 && temp!=38){
               sequence[i]=temp2;
             }
           }
           break;
        }
      }
     finals[0]=String(sequence[0])+String(sequence[1])+String(sequence[2]);
     finals[1]=String(sequence[3])+String(sequence[4])+String(sequence[5]);
     finals[2]=String(sequence[6])+String(sequence[7])+String(sequence[8]);
     finals[3]=String(sequence[9])+String(sequence[10])+String(sequence[11]);
     finals[4]=String(sequence[12])+String(sequence[13])+String(sequence[14]);
     
      Serial.println(finals[0]+" "+finals[1]+" "+finals[2]+" "+finals[3]+" "+finals[4]);
      SetPosition(servoNumOne,finals[0].toInt());
      SetPosition(servoNumTwo,finals[1].toInt());
      
      thrustOne.write(finals[2].toInt());
      thrustTwo.write(finals[3].toInt());
      thrustThree.write(finals[4].toInt());
      }
      else{
       Serial.println(Serial.read()); 
      }

    }

