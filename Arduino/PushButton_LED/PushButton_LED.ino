const int led_blue = 9;
const int led_red = 10;
const int led_green = 11;
const int button = 7;
int i = 0;
int val = 0;
int old_val = 0;
int state = 0;

int brightness = 128;
unsigned long startTime = 0;

void setup(){
  pinMode(led_blue, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(button, INPUT);
}

void loop(){
  val = digitalRead(button);
  if ((val == HIGH) && (old_val == LOW)){
    state = 1 - state;
    startTime = millis();
    delay(10);
  }
  if ((val == HIGH) && (old_val == HIGH)){
    if (state == 1 && (millis() - startTime) > 500){
      brightness++;
      delay(10);
      if (brightness > 255){
        brightness = 0;
      }
    }
  }

  old_val = val;

  if (state == 1){
    analogWrite(led_blue, brightness);
    analogWrite(led_red, brightness);
    analogWrite(led_green, brightness);
  } else {
    analogWrite(led_blue, 0);
    analogWrite(led_red, 0);
    analogWrite(led_green, 0);
  }
}
