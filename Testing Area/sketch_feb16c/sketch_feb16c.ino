int number = 0;

void setup(){
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  analogWrite(A16, 55);
}

void loop(){
  number = analogRead(A10);
  Serial.println("Loop");
  Serial.println(number);
  delay(759);
  
  if (analogRead(A10) > 500){
      digitalWrite(5, HIGH);
  }
  else{
     digitalWrite(5, LOW); 
  }
}
