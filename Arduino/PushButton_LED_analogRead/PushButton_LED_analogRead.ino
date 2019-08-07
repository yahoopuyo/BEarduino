const int led_blue = 9;
const int led_red = 10;
const int led_green = 11;

int val = 0;


void setup(){
  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
//analogIN端子は宣言しなくてもINPUT
  Serial.begin(9600);
}


void loop(){
  val = analogRead(0);
  //Serial.print(val);
  //Serial.print("\n"); 
  Serial.println(val);
  analogWrite(led_blue, val/4);
  analogWrite(led_red, val/4);
  analogWrite(led_green, val/4);
  delay(10);
}
