   #include <ax12.h>
   #include <Servo.h>
   Servo thrustOne;
   Servo thrustTwo;
   Servo thrustThree;
   int servoNumOne=0;//forgot which ones i set them as
   int servoNumTwo=15;//this number sounds familiar
void setup(){
   Serial.begin(38400); 

    //thrustOne.attach(1);//arbitrary pin#
    //thrustTwo.attach(2);//arbitrary pin#
    //thrustThree.attach(3);//arbitrary pin# 
}
void loop(){
    int i;
    int input[7];
    int temp;
    delay(500);
    int characters[15];
    Serial.print(Serial.available()+" ");
    if(Serial.peek()==45){
      Serial.read();
      while(1){
        if(Serial.available()==20){//will change to accomadate \n
           Serial.print("In the if");
           for (int i=0;i<15;i++){
             int temp=(int)Serial.read();
             if(temp!=42 && temp!=38){
               characters[i]=temp;
             }
             else{
              characters[i]=(int)Serial.read(); 
             }
           }
           break;
        }
      }
    }
    else{
     Serial.println(Serial.read()); 
    }
      int* inputs = arrayToInt(characters);
      SetPosition(servoNumOne,inputs[0]);
      SetPosition(servoNumTwo,inputs[1]);
      
      thrustOne.write(inputs[2]);
      thrustTwo.write(inputs[3]);
      thrustThree.write(inputs[4]);
    }
      
    
   int* arrayToInt(int list[]){
     String int1 = (String)list[0]+(String)list[1]+(String)list[2];
     int servoOne=int1.toInt();
     
     String int2 = (String)list[3]+(String)list[4]+(String)list[5];
     int servoTwo=int1.toInt();
     
     String int3 = (String)list[6]+(String)list[7]+(String)list[8];
     int thrustOne=int1.toInt();
     
     String int4 = (String)list[9]+list[10]+list[11];
     int thrustTwo=int1.toInt();
     
     String int5 = (String)list[12]+list[13]+list[14];
     int thrustThree=int1.toInt();
     
     int returnVal[5] = {servoOne, servoTwo, thrustOne, thrustTwo, thrustThree};
     return returnVal;
   }
