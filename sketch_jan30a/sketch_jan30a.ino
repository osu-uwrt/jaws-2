   
   
   
    int servoNumOne=0;//forgot which ones i set them as
    int servoNumTwo=15;//this number sounds familiar
void setup(){
    Serial.begin(38400); 

    
   // Servo thrustOne;
   // Servo thrustTwo;
   // Servo thrustThree;//Conor claims thrusters work this simply
    //thrustOne.attach(1);//arbitrary pin#
    //thrustTwo.attach(2);//arbitrary pin#
    //thrustThree.attach(3);//arbitrary pin# 
}
void loop(){
    int i;
    int input[6];
    while(Serial.available()>0){
      input = Serial.read();
      //implement delay later on with the math
      SetPosition(servoNumOne,input[0]);
      SetPosition(servoNumTwo,input[1]);
      
     // thrustOne.write(input[2]);
   //   thrustTwo.write(input[3]);
     // thrustThree.write(input[4]);
      
    }
    
    //Something nifty to think about:
    //Arbotix cant support 2 traditional PWM servos,
    //meaning our three thrusters might run into a problem with that
    
}
