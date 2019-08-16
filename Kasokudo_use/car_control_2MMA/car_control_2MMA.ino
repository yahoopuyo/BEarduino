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
 #define DelayTime 500       //speed of car
 const int speed_var_rot_pat_1 = 180;
 const int speed_var_rot_pat_2 = 180;
 const int speed_var_rot_pat_3 = 180;
 const int speed_var_rot_pat_4 = 180;
 const int speed_var_rot_pat_5 = 180;
 const int speed_var_rot_pat_6 = 180;
 const int speed_var_rot_pat_7 = 180;
 const int speed_var_rot_pat_8 = 180;
 const int speed_var_rot_pat_0 = 180;

 const int move_time_rot_pat_1 = 500;
 const int move_time_rot_pat_2 = 500;
 const int move_time_rot_pat_3 = 500;
 const int move_time_rot_pat_4 = 500;
 const int move_time_rot_pat_5 = 500;
 const int move_time_rot_pat_6 = 500;
 const int move_time_rot_pat_7 = 500;
 const int move_time_rot_pat_8 = 500;
 const int move_time_rot_pat_0 = 1000;


#define ACmeter_count 3  //加速度センサのtmp値を何回の平均値とするか
#define ACmeter_buf_count 10 //過去何回のtmp値を保存し、その平均をとるか
#define gForceX_threshold 0.02
#define gForceY_threshold 0.02
#define gForceZ_threshold 0.02
 
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

float xbuf[ACmeter_buf_count], ybuf[ACmeter_buf_count],zbuf[ACmeter_buf_count];
int counter = 0;
int cnt =0;
int flag = 0;

//センサー定義
Adafruit_MMA8451 mma = Adafruit_MMA8451();
 IRrecv IR(IR_PIN);  //   IRrecv object  IR get code from IR remoter
 decode_results IRresults;   

/***************motor control***************/

void go_Advance(int speed_var)  //Forward
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,speed_var);
  analogWrite(speedPinR,speed_var);
}
void go_Left(int speed_var)  //Turn left
{
  digitalWrite(dir1PinL, HIGH);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,speed_var);
  analogWrite(speedPinR,speed_var);
}
void go_Right(int speed_var)  //Turn right
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,HIGH);
  digitalWrite(dir2PinR,LOW);
  analogWrite(speedPinL,speed_var);
  analogWrite(speedPinR,speed_var);
 
}
void go_Back(int speed_var)  //Reverse
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,HIGH);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,HIGH);
  analogWrite(speedPinL,speed_var);
  analogWrite(speedPinR,speed_var);
 
}
void stop_Stop()    //Stop
{
  digitalWrite(dir1PinL, LOW);
  digitalWrite(dir2PinL,LOW);
  digitalWrite(dir1PinR,LOW);
  digitalWrite(dir2PinR,LOW);
}
void movement(int move_time)
{
      delay(move_time);
       stop_Stop();
}

/**************detect IR code***************/
/*
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
*/
/***********加速度センサからの入力に応じた処理***********/
float xaverage() {
  float avex = 0;
  for ( int s = 0; s < ACmeter_buf_count; s++){
    avex = avex + xbuf[s];
  }
  avex = avex/ACmeter_buf_count;
  return avex;
}
float yaverage() {
  float avey = 0;
  for ( int s = 0; s < ACmeter_buf_count; s++){
    avey = avey + ybuf[s];
  }
  avey = avey/ACmeter_buf_count;
  return avey;
}
float zaverage() {
  float avez= 0;
  for (int s =0; s < ACmeter_buf_count; s++){
    avez = avez + zbuf[s];
  }
  avez = avez/ACmeter_buf_count;
  return avez;
}

void do_ACmeter_move(){
  /*ここに、gForceX, gForceY, gForceZ, rotX, rotY, rotZの値に応じた処理を書く*/
  /*この加速度センサはx、y、z軸いずれも、鉛直上を向いているときに、+1gとなる。*/
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

  float d_gForceX = 0;
  float d_gForceY = 0;
  float d_gForceZ = 0;
  int rot_pat = 0;


    Serial.print("______________________________________________\n");
   
   
   for (int i=0; i<ACmeter_count; i++) {
    //Serial.print("i = ");
    //Serial.print(i);
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
   tmp_gForceX = tmp_gForceX / ACmeter_count;
   tmp_gForceY = tmp_gForceY / ACmeter_count;
   tmp_gForceZ = tmp_gForceZ / ACmeter_count;

   
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
  //Serial.print("counter = ");
  //Serial.print(counter);
  //Serial.print("  \n");
  //Serial.print("xbuf[counter] =");
  //Serial.print(xbuf[counter]);
  //Serial.print("  \n");
  counter++;
  
  if ( cnt < ACmeter_buf_count) cnt++;

  
  Serial.print("cnt = ");
  Serial.print(cnt);
  Serial.print("\n");


  
  if ( counter == ACmeter_buf_count) {
    counter = 0;
    flag = 1;
  }
  if (flag == 1) {
    //Serial.print("ここ");
//    mean_gForceX = xaverage();
//    mean_gForceY = yaverage();
//    mean_gForceZ = zaverage();

    mean_gForceX = -0.82;
    mean_gForceY = 0;
    mean_gForceZ = 0.51;
    
    Serial.print("mean_gForceX = ");
    Serial.print(mean_gForceX);
    Serial.print("    mean_gForceY = ");
    Serial.print(mean_gForceY);
    Serial.print("    mean_gForceZ = ");
    Serial.print(mean_gForceZ);
    Serial.print("  \n");
    
    
  }

  //まず、変化の差分をd_に代入。
      d_gForceX = tmp_gForceX - mean_gForceX;
      d_gForceY = tmp_gForceY - mean_gForceY;

      
      Serial.print("d_gForceX = ");
      Serial.print(d_gForceX);
      Serial.print("  d_gForceY = ");
      Serial.print(d_gForceY);
      Serial.print("\n");

      
  //先の変化から十分なデータ数を取ってから(cntがACmeter_buf_count分たまったら）、また変化したら、って意味の二重if文。
  if ( (d_gForceX>gForceX_threshold) || (d_gForceY>gForceY_threshold) ) { 
      if (cnt>=ACmeter_buf_count) {


      //ここから回転パターンの識別に入る。
      if (d_gForceX > 0 && d_gForceY > 0 && tmp_gForceX < 0) { //tmp_gForceXの正負を比較することで、rot_pat = 1と=6とを区別。以下同様。
        rot_pat = 1;
      } else if (d_gForceX > 0 && d_gForceY < 0 && tmp_gForceX > 0) {
        rot_pat = 2;
      } else if (d_gForceX < 0 && d_gForceY < 0 && tmp_gForceX > 0) {
        rot_pat = 3;
      } else if (d_gForceX < 0 && d_gForceY > 0 && tmp_gForceX < 0) {
        rot_pat = 4; 
      } else if (d_gForceX > 0 && d_gForceY < 0 && tmp_gForceX < 0) {
        rot_pat = 5;
      } else if (d_gForceX > 0 && d_gForceY > 0 && tmp_gForceX > 0) {
        rot_pat = 6;
      } else if (d_gForceX < 0 && d_gForceY > 0 && tmp_gForceX > 0) {
        rot_pat = 7;
      } else if (d_gForceX < 0 && d_gForceY < 0 && tmp_gForceX < 0) {
        rot_pat = 8;
      }
    }
   } else { //変化が検出されなかったら、回転パターンを0に戻す。
    if (cnt>=ACmeter_buf_count){
    //rot_pat = 0;
    }
   }

  switch (rot_pat) {
    case 1:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Right(speed_var_rot_pat_1); //回転スピードの設定
      movement(move_time_rot_pat_1); //移動時間の設定
      Serial.print("  GO RIGHT\n");
      break;
    case 2:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Right(speed_var_rot_pat_2); //回転スピードの設定
      movement(move_time_rot_pat_2); //移動時間の設定
      Serial.print("  GO RIGHT\n");
      break;    
    case 3:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Right(speed_var_rot_pat_3); //回転スピードの設定
      movement(move_time_rot_pat_3); //移動時間の設定
      Serial.print("  GO RIGHT\n");
      break;
    case 4:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Right(speed_var_rot_pat_4); //回転スピードの設定
      movement(move_time_rot_pat_4); //移動時間の設定
      Serial.print("  GO RIGHT\n");
      break;
    case 5:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Left(speed_var_rot_pat_5); //回転スピードの設定
      movement(move_time_rot_pat_5); //移動時間の設定
      Serial.print("  GO LEFT\n");
      break;
    case 6:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Left(speed_var_rot_pat_6); //回転スピードの設定
      movement(move_time_rot_pat_6); //移動時間の設定
      Serial.print("  GO LEFT\n");
      break;
    case 7:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Left(speed_var_rot_pat_7); //回転スピードの設定
      movement(move_time_rot_pat_7); //移動時間の設定
      Serial.print("  GO LEFT\n");
      break;
    case 8:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Left(speed_var_rot_pat_8); //回転スピードの設定
      movement(move_time_rot_pat_8); //移動時間の設定
      Serial.print("  GO LEFT\n");
      break;
    case 0:
      Serial.print("rot_pat =");
      Serial.print(rot_pat);
      Serial.print("\n");
      go_Advance(speed_var_rot_pat_0);
      movement(move_time_rot_pat_0); //移動時間の設定
      stop_Stop();    
      Serial.print("GO ADVANCE\n");
      break;
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

  // シリアルポートを9600 bps[ビット/秒]で初期化 
  Serial.begin(9600);

  //加速度センサを使うため
  Wire.begin();
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
