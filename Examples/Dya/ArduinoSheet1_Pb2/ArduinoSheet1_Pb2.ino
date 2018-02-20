void setup()
{
  pinMode(7, OUTPUT);   // sets the pin as output
  pinMode(4, OUTPUT);   // sets the pin as output
  pinMode(5, OUTPUT);   // sets the pin as output
  pinMode(6, OUTPUT);   // sets the pin as output

  Serial.begin(9600);

}

void loop()
{
  digitalWrite(4,HIGH); 
  delay(1000);
  digitalWrite(4,LOW); 
  digitalWrite(5,HIGH);
  delay(1000);
  digitalWrite(5,LOW); 
  digitalWrite(6,HIGH); 
  delay(1000);
  digitalWrite(6,LOW); 
  digitalWrite(7,HIGH); 
  delay(1000);
  digitalWrite(7,LOW); 

}
