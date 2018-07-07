

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
int velocity;
int trigPin = 11;
int echoPin = 12;
float lol = 0;

const int D = 6;


void setup() {
  Serial.begin(9600);
  //IR SETUP
  
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
void circle(int s,int z,int mode) {
  if (mode = 0 ) {
  analogWrite(ENA, s);
  analogWrite(ENB, z);
  }
  else {
  analogWrite(ENA, z);
  analogWrite(ENB, s);
  }
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




/////////////////////////////////////////////////////////////////

bool confirm_angle(int input_angle) {
/*// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
MPU6050 mpu;

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
float prev_ypr = 0;
bool calibrated = false;
bool desired_angle_calc = true;
int desired_angle = 0;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


 // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
    // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    ////////
    while(true) {
          // if programming failed, don't try to do anything
    if (!dmpReady) return;
      
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in 
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            if( prev_ypr == ypr[0]){
               calibrated = true;  
               
            }
            prev_ypr = ypr[0];
            if (claibrated == true) {
              if (desired_angle_calc) {
                desired_angle = (int)(ypr[0] * 180/M_PI) + input_angle;
                desired_angle_calc = false;
              }
              if(desired_angle > 180) {
                desired_angle = desired_angle - 360;
              }
              if(desired_angle < -180) {
                desired_angle = desired_angle + 360;
              }
              if((int)(ypr[0] * 180/M_PI) + 180 > desired_ angle + 180) {
                rotate_left(200);
              }
               if((int)(ypr[0] * 180/M_PI) + 180 < desired_ angle + 180) {
                rotate_right(200);
              }
              if((int)(ypr[0] * 180/M_PI) + 180 == desired_ angle + 180) {
                steady();
                break;
                }
            }
    }
}
mpu.setDMPEnabled(false);
calibrated = false;*/
}

////////////////////////////////////////////////////////////////






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
    Serial.print("Precise Movement Application Activated \n 'C' for circle \n 'S' for square \n 'A' for angle \n any key for distance \n");
    while (true){
      if(Serial.available() > 0)
      break;
    }
    if (Serial.available() > 0) {
    x = Serial.read();
      switch(x) {
        case 'C': 
        while (true){
          Serial.print("Press any key to start . .  \n");
          if(Serial.available() > 0){
           circle(255,90,0);
           while(true) {
            if (Serial.available() > 0 && Serial.read() == 'Z') {
              steady();
              break;
            }
           }
           break;
          }        
        }
        break;


        case 's': while (true){
         
          if(Serial.available() > 0){

       
            rotate_left(255);
            delay(425);
            steady();
            
           while(true) {
            if (Serial.available() > 0 && Serial.read() == 'Z') {
              steady();
              break;
            }
           }
           break;
          }        
        }
        break;



        
        case 'S': while (true){
          Serial.print("entered1 \n");
          int delayer= ( (1.1*0.5*200)/velocity ) * 1000;
          if(Serial.available() > 0){
            Serial.print("entered2 \n");
             forward(200);
            delay(delayer);
            steady();
            rotate_left(200);
             Serial.print("entered3 \n");
            delay(350);
            steady();
            forward(200);
            delay(delayer);
            steady();
            rotate_left(200);
            delay(350);
            steady();
            forward(200);
            delay(delayer);
            steady();
            rotate_left(200);
            delay(350);
            steady();
            forward(200);
            delay(delayer);
            steady();
            
           while(true) {
            if (Serial.available() > 0 && Serial.read() == 'Z') {
              steady();
              break;
            }
           }
           break;
          }        
        }
        break;



  case 'I': while (true){
          if(Serial.available() > 0){

           circle(255,30,0);
           delay(2000);
           circle(255,30,1);
           delay(2000);
          steady();
            
           while(true) {
            if (Serial.available() > 0 && Serial.read() == 'Z') {
              steady();
              break;
            }
           }
           break;
          }        
        }
        break;






 case 'J': while (true){
            
            /* we figured out from approximate calculations that the 90 degree takes about 425ms to do 90 degree rotation, so we used it to rotate the car at different degrees   */
            if (x == 'C')
            {
               rotate_left(200);
               delay(425);
               steady();
               x='$';
            }

            else if (x== 'U')
            {
              /*45*/
            rotate_left(200);
            delay(225);
            steady();
            x='$';
            }

            else if(x== 'E')
            {
              /*180*/
            rotate_left(200);
            delay(850);
             steady();
             x='$';
            }
            else
            {
              x='$';
              steady();
            }
            
          while(true) {
            if (Serial.available() > 0 && Serial.read() == 'Z') {
              steady();
              break;
            }
           }
           break;
          }        
        
        break;







        
        case 'A':Serial.print("Please Enter The Angle in degrees \n"); 
        while (true){
          if(Serial.available() > 0){
            int d = Serial.parseInt();
            Serial.print(d);
            confirm_angle(d);
            break;

            }

          }
        break;
        default: Serial.print("Please Enter The Distance in cm \n");  
        while (true){
          if(Serial.available() > 0){
            int d = Serial.parseInt();
            Serial.print(d);
            // Velocity
            Serial.print(d/velocity);
            float time00 = ((float)d/(float)velocity)* 1000;
            forward(200);
            delay(time00);
            steady();
            //IR Mode
            /*d = d/5;
            forward(200);
            rotate_wheels((int)d);
            steady();*/
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
     
    // UltraSonic Mode to get velocity
    
       float z = get_distance(trigPin,echoPin);
       Serial.print("Distance \n");
       Serial.print(z);
        delay(1000);
       forward(200);
       delay(1000);
       steady();
       delay(1000);
       z = z - get_distance(trigPin,echoPin);
       velocity = z;
       Serial.print("Velocity = ");
       Serial.print(velocity); 
       Serial.print("\n Calibration Terminated \n");

   }
   
  if (x == '0') {
        steady();
      }
  }
  }

 

