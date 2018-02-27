int led1 = 4;
int led2 = 5;
int led3 = 6;
int led4 = 7;


void setup()
  
{
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
}

void loop()
{
  digitalWrite(led1, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led1, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led2, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led2, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led3, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led3, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led4, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led4, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
   digitalWrite(led3, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led3, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
    digitalWrite(led2, HIGH);
  delay(1000); // Wait for 1000 millisecond(s)
  digitalWrite(led2, LOW);
  delay(1000); // Wait for 1000 millisecond(s)
}