/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/ 
 * Arduino Smart Car Tutorial Lesson 4
 * Tutorial URL http://osoyoo.com/2018/12/19/osoyoo-robot-car-kit-lesson-4-tracking-line-robot-car/
 * CopyRight www.osoyoo.com

 * This lesson will let robot car automatically track a black line in ground
 *  
 * 
 */


/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
#define L298N Dual H-Bridge Motor Controller Pins
#define speedPinR 46 // RIGHT PWM pin connect MODEL-X ENA, was 9
#define RightDirectPin1 48
#define RightDirectPin2 50
#define speedPinL 47 // Left PWM pin connect MODEL-X ENB, was 6
#define LeftDirectPin1 49
#define LeftDirectPin2 51 //Left Motor direction pin 1 to MODEL-X IN4, was 8

#include <IRremote.h>  
#define IR_PIN    10 //IR receiver Signal pin connect to Arduino pin D2 
 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;   
//
//#define speedPinR 9    //  RIGHT PWM pin connect MODEL-X ENA
//#define RightDirectPin1  12    //Right Motor direction pin 1 to MODEL-X IN1 
//#define RightDirectPin2  11    //Right Motor direction pin 2 to MODEL-X IN2
//#define speedPinL 6    // Left PWM pin connect MODEL-X ENB
//#define LeftDirectPin1  7    //Left Motor direction pin 1 to MODEL-X IN3 
//#define LeftDirectPin2  8   //Left Motor direction pin 1 to MODEL-X IN4 

/*From left to right, connect to D3,A1-A3 ,D10*/
#define LFSensor_0 A0
#define LFSensor_1 A1
#define LFSensor_2 A2
#define LFSensor_3 A3
#define LFSensor_4 A4

#define IR_ADVANCE       0x00FF18E7       //code from IR controller "▲" button
 #define IR_BACK          0x00FF4AB5       //code from IR controller "▼" button
 #define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
 #define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
 #define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
 #define IR_HASH     0x00FFB04F       //code from IR controller "#" button
 #define IR_STAR     0x00FF6897       //code from IR controller "*" button
 #define IR_one      0x00000001                     // code for IR controller '1" button
#define IR_two         0x00000002                 // code for IR controller '2" butto
#define IR_three       0x00000003                     // code for IR controller '3" button
#define IR_four       0x00000004                     // code for IR controller '4" button
#define IR_five      0x00000005                      // code for IR controller '5" button
#define IR_six       0x00000006                   // code for IR controller '6" button
#define IR_seven      0x00000007                    // code for IR controller '7" button
#define IR_eight      0x00000008                     // code for IR controller '8" button
#define IR_nine      0x00000009                     // code for IR controller '9" button
#define IR_zero      0x00000000                      // code for IR controller '0" button

#define SPEED   180 //motor in   speed



 
 
/* defines values for target row /column in grid  */

int targetrow = 0;
int targetcolumn = 0;
int currentrow = 0;
int currentcolumn = 0;
int currentIRValue = 0;



enum DN
{ 
  GO_ADVANCE, //go forward
  GO_LEFT, //left turn
  GO_RIGHT,//right turn
  GO_BACK,//backward
  STOP_STOP, 
  HASH,
  STAR,
  DEF
}Drive_Num=DEF;


int coordinates[3][4]; //stores the four coordinates

/**************detect IR code***************/
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_ADVANCE)
    {
      Drive_Num=GO_ADVANCE;
    }
    else if(IRresults.value==IR_RIGHT)
    {
       Drive_Num=GO_RIGHT;
    }
    else if(IRresults.value==IR_LEFT)
    {
       Drive_Num=GO_LEFT;
    }
    else if(IRresults.value==IR_BACK)
    {
        Drive_Num=GO_BACK;
    }
    else if(IRresults.value==IR_STOP)
    {
        Drive_Num=STOP_STOP; 
    }
    else if(IRresults.value==IR_STAR)
    {
        Drive_Num=STAR; 
    }
    else if(IRresults.value==IR_HASH)
    {
        Drive_Num=HASH; 
    }
    else if(IRresults.value== IR_zero)
    {
      currentIRValue = 0;
    }
    else if(IRresults.value== IR_one)
    {
      currentIRValue = 1;
    }
    else if(IRresults.value== IR_two)
    {
      currentIRValue = 2;
    }
    else if(IRresults.value== IR_three)
    {
      currentIRValue = 3;
    }
    else if(IRresults.value== IR_four)
    {
      currentIRValue = 4;
    }
    else if(IRresults.value== IR_five)
    {
      currentIRValue = 5;
    }
    else if(IRresults.value== IR_six)
    {
      currentIRValue = 6;
    }
    else if(IRresults.value== IR_seven)
    {
      currentIRValue = 7;
    }
    else if(IRresults.value== IR_eight)
    {
      currentIRValue = 8;
    }
    else if(IRresults.value== IR_nine)
    {
      currentIRValue = 9;
    }
    
 
   IRresults.value = 0;  //reset
    IR.resume();
  }
}

 char sensor[5];
 /*read sensor value string, 1 stands for black, 0 starnds for white, i.e 10000 means the first sensor(from left) detect black line, other 4 sensors detected white ground */
String read_sensor_values()
{   int sensorvalue=32;
  sensor[0]= digitalRead(LFSensor_0);
 
  sensor[1]=digitalRead(LFSensor_1);
 
  sensor[2]=digitalRead(LFSensor_2);
 
  sensor[3]=digitalRead(LFSensor_3);
 
  sensor[4]=digitalRead(LFSensor_4);
  sensorvalue +=sensor[0]*16+sensor[1]*8+sensor[2]*4+sensor[3]*2+sensor[4];
 
  String senstr= String(sensorvalue,BIN);
  Serial.println(senstr);
  return senstr.substring(1,6);
}

void stop_Idle() //Stop but coast
{
Serial.println ("stop_Idle");
analogWrite(speedPinL,0);
analogWrite(speedPinR,0);
Serial.println("Idle stop.");
}
void go_Advance(void)  //Forward
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  Serial.println("Go advance.");
}
void go_Left(void)  //Turn left
{
  digitalWrite(RightDirectPin1, HIGH);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  Serial.println("Go left.");
}

void go_Left_Coast(void)  //Turn left with coast
{
  digitalWrite(RightDirectPin1, LOW);  //disable front right wheel
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);    
  digitalWrite(LeftDirectPin2,HIGH);
  Serial.println("Coast left.");
}


void go_Right(void)  //Turn right 
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,HIGH);
  digitalWrite(LeftDirectPin2,LOW);
  Serial.println("Go right.");
}

void go_Right_Coast(void)  //Turn right with coast
{
  digitalWrite(RightDirectPin1, LOW);  
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);  //disable front left wheel
  digitalWrite(LeftDirectPin2,LOW);
  Serial.println("Coast right.");
}


void go_Back(void)  //Reverse
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,HIGH);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,HIGH);
  Serial.println("Go back.");
}
void stop_Stop()    //Stop
{
  digitalWrite(RightDirectPin1, LOW);
  digitalWrite(RightDirectPin2,LOW);
  digitalWrite(LeftDirectPin1,LOW);
  digitalWrite(LeftDirectPin2,LOW);
  Serial.println("Stop.");
}
/*set motor speed */
void set_Motorspeed(int speed_L,int speed_R)
{
  analogWrite(speedPinL,speed_L); 
  analogWrite(speedPinR,speed_R);   
}

void setup()
{
 pinMode(speedPinL,OUTPUT); //left motor PWM pin
 pinMode(speedPinR,OUTPUT); //rignt motor PWM  pin
 pinMode(RightDirectPin1,OUTPUT); //left motor direction pin1
 pinMode(RightDirectPin2,OUTPUT); //left motor direction pin2
 pinMode(LeftDirectPin1,OUTPUT); //right motor direction Pin 1
 pinMode(LeftDirectPin2,OUTPUT);  //right motor direction Pin 2

  /*line follow sensors */
 pinMode(LFSensor_0,INPUT);
 pinMode(LFSensor_1,INPUT);
 pinMode(LFSensor_2,INPUT);
 pinMode(LFSensor_3,INPUT);
 pinMode(LFSensor_4,INPUT); 
 Serial.begin(9600);
}

void auto_tracking(){
 String sensorval= read_sensor_values();
 //Serial.println(sensorval);

 //only use middle sensor  -igonre other sensors to follow grid line
 // if in white space keep moving until you hit the blackline
 if ( sensorval !="00100" )
 { 
    go_Advance();  //go straight
    set_Motorspeed(0,SPEED);
    delay(200);
    stop_Stop();
    }
 if (sensorval=="11111" or sensorval=="01111" or sensorval=="11110")
 {
      Serial.println("The front of the car is touching the stop line.");    
     stop_Stop();   //The car front touch stop line, need stop
     set_Motorspeed(0,0);
    }
}

void loop(){

  //current program assumes user gives the values in the exact order without mistakes  colno, *, rowno, #, direction
  //loops 4 times to get 4 set of coordiantes and stores it in the array
   
  
  for (int i=0; i<4;i++)
  {
  do_IR_Tick();  coordinates[0][0] = currentIRValue;
  do_IR_Tick(); //ignore the * which is in input value
  do_IR_Tick; coordinates[0][1] = currentIRValue;
  do_IR_Tick; //ignore the #value;
  do_IR_Tick; //ignore the dreiction for now -always assume it goes straight in the direction it is placed.

  }


//traverse the 4 coordinates   

for (int i=0; i<5;i++)
{
   targetcolumn = coordinates[i][0];  //get the set of coordinates
   targetrow = coordinates[i][1];

//moves to the target column

for (int j = 1; i <  targetcolumn; i++)
{ 
   auto_tracking();
}
//turns left so it is facing correctly to move along rows
go_Left();
//moves to the target row
for (int i = 1; i < targetrow; i++)
{
    auto_tracking();
}
} //finish going thru all the 4 coordinates


}
