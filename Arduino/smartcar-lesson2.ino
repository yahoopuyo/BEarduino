
/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/    
 * www.osoyoo.com IR remote control smart car
 * program tutorial : http://osoyoo.com/2017/04/16/control-smart-car-with-ir/
 *  Copyright John Yu
 */
#include <IRremote.h>
/*Declare L298N Dual H-Bridge Motor Controller directly since there is not a library to load.*/
//Define L298N Dual H-Bridge Motor Controller Pins
#define dir1PinL  2    //Motor direction
#define dir2PinL  4    //Motor direction
#define speedPinL 6    // Needs to be a PWM pin to be able to control motor speed

#define dir1PinR  7    //Motor direction
#define dir2PinR  8   //Motor direction
#define speedPinR 5    // Needs to be a PWM pin to be able to control motor speed

#define IR_PIN    3 //IR receiver Signal pin connect to Arduino pin 3

 #define IR_ADVANCE       0x00FF18E7       //code from IR controller "▲" button
 #define IR_BACK          0x00FF4AB5       //code from IR controller "▼" button
 #define IR_RIGHT         0x00FF5AA5       //code from IR controller ">" button
 #define IR_LEFT          0x00FF10EF       //code from IR controller "<" button
 #define IR_STOP          0x00FF38C7       //code from IR controller "OK" button
 #define IR_turnsmallleft 0x00FFB04F       //code from IR controller "#" button
 #define SPEED 180       //speed of car
 #define DelayTime 2500       //speed of car
 
enum DN
{ 
  GO_ADVANCE, //go forward
  GO_LEFT, //left turn
  GO_RIGHT,//right turn
  GO_BACK,//backward
  STOP_STOP, 
  DEF
}Drive_Num=DEF;

bool stopFlag = true;//set stop flag
bool JogFlag = false;
uint16_t JogTimeCnt = 0;
uint32_t JogTime=0;
uint8_t motor_update_flag = 0;
                                                                                            
 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;   

/***************motor control***************/

void go_Advance(void)  //Forward
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,SPEED);
  analogWrite(speedPinR,SPEED);
}
void go_Left()  //Turn left
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,SPEED);
  analogWrite(speedPinR,SPEED);
}
void go_Right(int t=0)  //Turn right
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,SPEED);
  analogWrite(speedPinR,SPEED);
 
}
void go_Back(int t=0)  //Reverse
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,SPEED);
  analogWrite(speedPinR,SPEED);
 
}
void stop_Stop()    //Stop
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
}
void movement()
{
      delay(DelayTime);
       stop_Stop();
}
/**************detect IR code***************/
void do_IR_Tick()
{
  if(IR.decode(&IRresults))
  {
    if(IRresults.value==IR_ADVANCE)
    {
      go_Advance();
		movement();
    }
    else if(IRresults.value==IR_RIGHT)
    {
       go_Right();
		movement();
 
    }
    else if(IRresults.value==IR_LEFT)
    {
       go_Left();
		movement();
    }
    else if(IRresults.value==IR_BACK)
    {
         go_Back();
		movement();
    }
    else if(IRresults.value==IR_STOP)
    {
        stop_Stop();
    }
    IRresults.value = 0;
    IR.resume();
  }
}

/**************car control**************/
 
void setup()
{
  pinMode(dir1PinL, OUTPUT); 
  pinMode(dir2PinL, OUTPUT); 
  pinMode(speedPinL, OUTPUT);  
  pinMode(dir1PinR, OUTPUT);
  pinMode(dir2PinR, OUTPUT); 
  pinMode(speedPinR, OUTPUT); 
  stop_Stop();

  pinMode(IR_PIN, INPUT); 
  digitalWrite(IR_PIN, HIGH);  
  IR.enableIRIn();       
}


void loop()
{
  do_IR_Tick();
}
