int one=4,two=5,three=6,four=7;
void setup ()
{
pinMode(one,OUTPUT);
pinMode(two,OUTPUT);
pinMode(three,OUTPUT);
pinMode(four,OUTPUT);
}
void loop ()
{
digitalWrite(one,HIGH);
delay(1000);
digitalWrite(one,LOW);
digitalWrite(two,HIGH);
delay(1000);
digitalWrite(two,LOW);
digitalWrite(three,HIGH);
delay(1000);
digitalWrite(three,LOW);
digitalWrite(four,HIGH);
delay(1000);
digitalWrite(four,LOW);
digitalWrite(three,HIGH);
delay(1000);
digitalWrite(three,LOW);
digitalWrite(two,HIGH);
delay(1000);
digitalWrite(two,LOW);
}