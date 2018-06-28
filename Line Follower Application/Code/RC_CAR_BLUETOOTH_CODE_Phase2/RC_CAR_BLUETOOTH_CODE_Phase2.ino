int IN1 = 8;
int IN2 = 7;
int IN3 = 5;
int IN4 = 4;
int ENA = 9;
int ENB = 3;
int RIR =2;
int CIR =6;
int state=0;
int LIR =10;  
int speed = 255;
int calibDist= 100;
int velocity;
int trigPin = 11;
int echoPin = 12;
float lol = 0;

////
const int D = 6;
const int dataIN = 13; //IR sensor INPUT
unsigned long prevmillis; // To store time
unsigned long duration; // To store time difference
int rpm; // RPM value
boolean currentstate; // Current state of IR input scan
boolean prevstate; // State of IR sensor in previous scan


void setup() {
  Serial.begin(9600);
  //IR SETUP
  pinMode(dataIN,INPUT);       
  prevmillis = 0;
  prevstate = LOW;
  
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
  Serial.print("Please Choose your Application, 'E' for Easy Driving Application , 'U' for Line Follower Application, 'P' for Precise Movement, 'C' for Calibration \n");
  char x = Serial.read();
  if (x=='E'){ //Easy Driving Application
    Serial.print("Easy Driving Application Activated \n");
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
    Serial.print("Easy Driving Application Terminated \n");

  }
  else if (x=='U') { //Line Tracking Application
    Serial.print("Line Follower Application Activated \n");
    speed = 90;
    x = Serial.read();
    while(x!='Z') {
      x = Serial.read();
      int c = digitalRead(CIR);
      int l = digitalRead(LIR);
      int r = digitalRead(RIR);

      if (c ==  HIGH) {
        forward(speed);
        state='f';
      }
      else {
        if (l ==  HIGH) {
        rotate_left(180);
        state='l';
        }
        else if (r == HIGH) {
        rotate_right(180);
        state='r';
        }
        else{
          switch(state) {
            case 'f': forward(speed);
            break;
            case 'l': rotate_left(180);
            break;
            case 'r': rotate_right(180);
            break;
          }
        }
     }

   }
     Serial.print("Line Follower Application Terminated \n");

  }
  else if (x=='P') { //Precise Movement Application Activated
    Serial.print("Precise Movement Application Activated \n 'C' for circle \n 'S' for square \n any key for distance \n");
    while (true){
      if(Serial.available() > 0)
      break;
    }
    if (Serial.available() > 0) {
    x = Serial.read();
      switch(x) {
        case 'C':
        break;
        case 'S':
        break;
        default: Serial.print("Please Enter The Distance in cm \n");  
        while (true){
          if(Serial.available() > 0){
            int d = Serial.parseInt();
            Serial.print(d);
            Serial.print(d/velocity);
            float time00 = (d/velocity)* 1000;
            forward(255);
            delay(time00);
            steady();
            break; 
          }
         
        }
        
    }
    }
   
    Serial.print("Precise Movement Application Terminated \n");

  }
  else if (x=='C') { //Calibration Activated
    Serial.print("Calibration Activated \n");
    x = Serial.read();
    forward(255);
    while(x!='Z') {
      x = Serial.read();
      // RPM Measurement
      currentstate = digitalRead(dataIN); // Read IR sensor state
 if( prevstate != currentstate) // If there is change in input
   {
     if( currentstate == HIGH ) // If input only changes from LOW to HIGH
       {
        Serial.print("Change Found . . .");
         duration = ( micros() - prevmillis ); // Time difference between revolution in microsecond
         rpm = (60000000/duration); // rpm = (1/ time millis)*1000*1000*60;
         prevmillis = micros(); // store time for nect revolution calculation
       }
   }
  prevstate = currentstate; // store this scan (prev scan) data for next scan
  
   }
   steady();
    velocity = 3.14159*(D)*(rpm/60); // speed in cm/s
    Serial.print("Velocity = ");
    Serial.print(velocity);
    Serial.print("\n Calibration Terminated \n");
  }
  if (x == '0') {
        steady();
      }
  }
  }

 

