#include <ir_Lego_PF_BitStreamEncoder.h>
#include <boarddefs.h>
#include <IRremoteInt.h>
#include <IRremote.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>


/*  ___   ___  ___  _   _  ___   ___   ____ ___  ____  
 * / _ \ /___)/ _ \| | | |/ _ \ / _ \ / ___) _ \|    \ 
 *| |_| |___ | |_| | |_| | |_| | |_| ( (__| |_| | | | |
 * \___/(___/ \___/ \__  |\___/ \___(_)____)___/|_|_|_|
 *                  (____/    
 * www.osoyoo.com IR remote control smart car
 * program tutorial : http://osoyoo.com/2017/04/16/control-smart-car-with-ir/
 *  Copyright John Yu
 */
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
 #define DelayTime 500       //speed of car
 
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

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
 
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;

int xbuf[40], ybuf[40],zbuf[40];
int counter = 0;
int cnt =0;
int flag = 0;

//センサー定義
Adafruit_MMA8451 mma = Adafruit_MMA8451();
                                                                                            
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
    //movement(); //前進の移動時間を長くするため、これはコメントアウト
    delay(2000);
    stop_Stop();    
    Serial.print("GO ADVANCE\n");
    }
    else if(IRresults.value==IR_RIGHT)
    {
       go_Right();
    movement();
     Serial.print("GO RIGHT\n");
    }
    else if(IRresults.value==IR_LEFT)
    {
       go_Left();
    movement();
    Serial.print("GO LEFT\n");
    }
    else if(IRresults.value==IR_BACK)
    {
         go_Back();
    movement();
    Serial.print("GO BACK\n");
    }
    else if(IRresults.value==IR_STOP)
    {
        stop_Stop();
    }
    IRresults.value = 0;
    IR.resume();
  }
}

/***********加速度センサからの入力に応じた処理***********/
int xaverage() {
  int avex = 0;
  for ( int s = 0; s < 40; s++){
    avex = avex + xbuf[s];
  }
  avex = avex/40;
  return avex;
}
int yaverage() {
  int avey = 0;
  for ( int s = 0; s < 40; s++){
    avey = avey + ybuf[s];
  }
  avey = avey/40;
  return avey;
}
int zaverage() {
  int avez= 0;
  for (int s =0; s < 40; s++){
    avez = avez + zbuf[s];
  }
  avez = avez/40;
  return avez;
}

void do_ACmeter_move(){
  /*ここに、gForceX, gForceY, gForceZ, rotX, rotY, rotZの値に応じた処理を書く*/
  /*姿勢は、車が鉛直壁に張り付いているとして、z軸方向が壁の法線方向と考えられるので、加速度のx軸、y軸成分(gForceX, gForeceY)を読み取り、それらの変化から姿勢を制御することを目標とする。*/
  /*進行パターンを縦移動か横移動かで制御が変わるのか。*/
  /*それとも、過去フレームからのずれを検知し、それを小さくするという制御を書くのか。*/
  /*今回は後者の方を目指すものとし、コードを書く。*/
  float tmp_gForceX = 0;
  float tmp_gForceY = 0;
  float tmp_gForceZ = 0;
  float mean_gForceX = 0;
  float mean_gForceY = 0;
  float mean_gForceZ = 0;


    Serial.print("______________________________________________\n");
   for (int i=0; i<5; i++) {
    Serial.print("i = ");
    Serial.print(i);
    recordAccelRegisters();
    Serial.print("  Accel");
    Serial.print("    X=");
    Serial.print(gForceX);
    Serial.print('g');
    Serial.print("    Y=");
    Serial.print(gForceY);
    Serial.print('g');
    Serial.print("    Z=");
    Serial.print(gForceZ);
    Serial.println('g');
    tmp_gForceX = tmp_gForceX + gForceX;
    tmp_gForceY = tmp_gForceY + gForceY;
    tmp_gForceZ = tmp_gForceZ + gForceZ;
   }
   tmp_gForceX = tmp_gForceX / 5;
   tmp_gForceY = tmp_gForceY / 5;
   tmp_gForceZ = tmp_gForceZ / 5;
   Serial.print("tmp_gForceX = ");
   Serial.print(tmp_gForceX);
   Serial.print("    tmp_gForceY = ");
   Serial.print(tmp_gForceY);
   Serial.print("    tmp_gForceZ = ");
   Serial.print(tmp_gForceZ);
   Serial.print("  \n");

  xbuf[counter] = tmp_gForceX;
  ybuf[counter] = tmp_gForceY;
  zbuf[counter] = tmp_gForceZ;
  counter++;
  //Serial.print("counter = ");
  //Serial.print(counter);

  if ( cnt < 40) cnt++;

  if ( counter == 40) {
    counter = 0;
    flag = 1;
  }
  if (flag == 1) {
    //Serial.print("ここ");
    mean_gForceX = xaverage();
    mean_gForceY = yaverage();
    mean_gForceZ = zaverage();
    Serial.print("mean_gForceX = ");
    Serial.print(mean_gForceX);
    Serial.print("    mean_gForceY = ");
    Serial.print(mean_gForceY);
    Serial.print("    mean_gForceZ = ");
    Serial.print(mean_gForceZ);
    Serial.print("  \n");
  }
  if ( (abs(tmp_gForceX - mean_gForceX)>0.05) || (abs(tmp_gForceY - mean_gForceY)>0.05) || (abs(tmp_gForceZ - mean_gForceZ)>0.05)) { 
    if (mean_gForceX != 0) {
      if (cnt>=40) {
      Serial.print("変化した！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！");
      cnt = 0;
      }
    }
   }

}


/**************加速度センサのため*************/
 
void setupMMA8451() {
  Serial.begin(9600);
  
  Serial.println("Adafruit MMA8451");
  

  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());
  Serial.println("G");
}
 
void recordAccelRegisters() {
  //加速度読み取り
  sensors_event_t event; 
  mma.getEvent(&event);
  accelX = event.acceleration.x;
  accelY = event.acceleration.y;
  accelZ = event.acceleration.z;
  calculateAccelData();
}
 
void calculateAccelData() {
  //読み取った値をgに変換
  gForceX = accelX / 9.80;
  gForceY = accelY / 9.80;
  gForceZ = accelZ / 9.80;
}
 
 
//void printData() {
//  //シリアルモニタに出力
//  Serial.print("  Accel");
//  Serial.print(" X=");
//  Serial.print(gForceX);
//  Serial.print('g');
//  Serial.print(" Y=");
//  Serial.print(gForceY);
//  Serial.print('g');
//  Serial.print(" Z=");
//  Serial.print(gForceZ);
//  Serial.println('g');
//}
/************加速度センサのため***************/
 
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

  //加速度センサを使うため
  setupMMA8451();

  
  
}

/*********リモコンからの赤外線命令ではなく、加速度センサの値を読み取り、それに対応した動作を行うようにしよう。************/
/*void loop()
{
  do_IR_Tick();
  recordAccelRegisters();
  recordGyroRegisters();
  printData();
  delay(200);//このdelayは小さくするべきか
}*/


void loop(){
  do_ACmeter_move();  
  //delay(5);//このdelayは小さくするべきか
}
