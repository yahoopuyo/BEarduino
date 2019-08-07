const int led_blue = 9;
const int led_red = 10;
const int led_green = 11;
int i = 0;

void setup(){
  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
}

void loop(){
  for (i = 0; i < 255; i++){
    analogWrite(led_blue, i );
    analogWrite(led_red, i);
    analogWrite(led_green, i);
    delay(10);
  }
  for ( i = 255; i > 0; i--){
    analogWrite(led_blue, i);
    analogWrite(led_red, i);
    analogWrite(led_green, i);
    delay(10);
  }
}
