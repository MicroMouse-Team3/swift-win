void setup(){
 pinMode(3, OUTPUT); 
}

void loop(){
  digitalWrite(3, HIGH);
  delay(50);
  digitalWrite(3,LOW);
  delay(50);
}
