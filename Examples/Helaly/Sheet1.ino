int IN4 = 4;
int IN5 = 5;
int IN6 = 6;
int IN7 = 7;



void setup() {
 
  pinMode(IN4,OUTPUT);
  pinMode(IN5,OUTPUT);
  pinMode(IN6,OUTPUT);
  pinMode(IN7,OUTPUT);
}



void loop() {
  
digitalWrite(IN4,HIGH);
delay(1000);
digitalWrite(IN4,LOW);
digitalWrite(IN5,HIGH);
delay(1000);
digitalWrite(IN5,LOW);
digitalWrite(IN6,HIGH);  
delay(1000);
digitalWrite(IN6,LOW);
digitalWrite(IN7,HIGH);
delay(1000);
digitalWrite(IN7,LOW);
digitalWrite(IN6,HIGH);
delay(1000);
digitalWrite(IN6,LOW);
digitalWrite(IN5,HIGH);
delay(1000);
digitalWrite(IN5,LOW);
digitalWrite(IN4,HIGH);
delay(1000);

  }
  
