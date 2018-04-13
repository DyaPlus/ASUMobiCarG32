int IN1 = 8;
int IN2 = 7;
int IN3 = 5;
int IN4 = 4;
int ENA = 9;
int ENB = 3;
int RIR =2;
int CIR =6;
int LIR =10;  
int speed = 255;
int trigPin = 11;
int echoPin = 12;
float lol = 0;
void setup() {
  Serial.begin(9600);
  //PINMODE SETUP
  pinMode(trigPin,OUTPUT);
  pinMode(echoPin,INPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (ENA, OUTPUT);
  pinMode (ENB, OUTPUT);
  pinMode(CIR,INPUT);
  pinMode(LIR,INPUT);
  pinMode(RIR,INPUT);


}

void forward(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void backward(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void rotate_right(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
void rotate_left(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void max_right(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void max_left(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void steering_right(int s) {
  analogWrite(ENA, s);
  analogWrite(ENB, s/2);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void steering_left(int s) {
  analogWrite(ENA, s/2);
  analogWrite(ENB, s);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}
void steady() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void rotate_left_us(int speed) {
  max_left(speed);
}
void rotate_right_us(int speed) {
  max_right(speed);
}

float get_distance(int trigPin,int echoPin) {
  float distance = 0;
  float duration = 0;
  //Clears the trig pin
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  //Generate signal and receive
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration=pulseIn(echoPin, HIGH);
  //Calc Distance
  distance = duration*0.034/2;
  return distance;
}
bool checkObstacle(int trigPin , int echoPin) {
  float distance = 0;
  float duration = 0;
  //Clears the trig pin
  digitalWrite(trigPin,LOW);
  delayMicroseconds(2);
  //Generate signal and receive
  digitalWrite(trigPin,HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin,LOW);
  duration=pulseIn(echoPin, HIGH);
  //Calc Distance
  distance = duration*0.034/2;
  lol = distance;
  if (distance <= 12 ) {
    return true;
  }
  else {
    return false;
  }
}

void loop() {
    
  if (Serial.available() > 0 ) {
   char  x=Serial.read();
  if (x=='E'){ //Easy Driving Application
    while ( x!='Z') {
      x=Serial.read();
      while (x == 'F') {
         if (Serial.available() > 0 ) 
          x=Serial.read();
       if (checkObstacle(trigPin,echoPin)) {
      steady();
      Serial.print(lol);
      Serial.print("\n");
     }
     else 
        forward(speed);
      }
       if (x == 'B') {
        backward(speed);
      }
       if (x == 'O') {
        rotate_right(speed);
      }
        if (x == 'I') {
        rotate_left(speed);
      }
        if (x == 'R') {
        max_right(speed);
      }
        if (x == 'L') {
        max_left(speed);
      }
      if (x ==  'Q') {
        steering_left(speed);
      }
       if (x ==  'G') {
        forward(speed);
      }
      if (x == 'E') {
        steering_right(speed);
      }
      if (x == '1') {
        speed = 128;
      }
         if (x == '2') {
        speed = 180;
      }
        if (x == '3') {
        speed = 255;
      }
     if (x == '0') {
        steady();
      }
    }
  }
  else if (x=='U') { //Line Tracking Application
    speed = 90;
    x = Serial.read();
    while(x!='Z') {
      x = Serial.read();
      int c = digitalRead(CIR);
      int l = digitalRead(LIR);
      int r = digitalRead(RIR);

      if (c ==  HIGH) {
        Serial.print("loool222");

        forward(speed);
      }
      else {
        if (l ==  HIGH) {
        rotate_left(speed);
        }
        if (r == HIGH) {
        rotate_right(speed);
        }
     }

   }
  }
  if (x == '0') {
        steady();
      }
  }
  }

 

