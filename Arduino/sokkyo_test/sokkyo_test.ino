void setup(){
  pinMode(0, OUTPUT);
  Serial.begin(9600);
}

void loop(){
  int a_in;
  int distance;
  a_in = analogRead(0);
  distance = (6762/(a_in-9))-4;
  Serial.println(distance);
}
