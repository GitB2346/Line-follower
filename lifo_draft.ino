// Motor driver pins
int in0 = 6;
int in1 = 5;
int in2 = 9;
int in3 = 10;


// int ena = 25;   // PWM pin
// int enb = 26;   // PWM pin

// IR sensors
#define S1 A4
#define S2 A3
#define S3 A2
#define S4 A1
#define S5 A0

// Encoder pins
#define ENC_L 2
#define ENC_R 3

volatile long leftCount = 0;
volatile long rightCount = 0;

// 90-degree turn calibration (START VALUE)
#define TURN_COUNT 260
#define FORWARD_COUNT 240



// int encoderPin1= 33;
// int encoderPin2 = 34;

//PID Varables
float Kp = 100;
float Ki = 0;
float Kd = 0;

float error = 0;
float previousError = 0;
float integral = 0;
unsigned long lastTime = 0;
int baseSpeed = 100;
int previous_error = 0;

//need to check for high and low
void setup() {
  // put your setup code here, to run once:

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in0, OUTPUT);

  
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);
  Serial.begin(9600);
    
  pinMode(ENC_L, INPUT_PULLUP);
  pinMode(ENC_R, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R), rightEncoderISR, RISING);


  lastTime = millis();



//   pinMode(encoderPin1, INPUT_PULLUP); 
//   pinMode(encoderPin2, INPUT_PULLUP);

//   digitalWrite(encoderPin1, HIGH); 
//   digitalWrite(encoderPin2, HIGH);


// attachInterrupt(encoderPin1, updateEncoder, CHANGE);
// attachInterrupt(encoderPin2, updateEncoder, CHANGE);


}
 int readError(int prev_error) {
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);   //check after serial printing 
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);
  // Serial.print(s1);
  // Serial.print(s2);
  // Serial.print(s3);
  // Serial.print(s4);
  // Serial.println(s5);
  

  int error =  (-2*s1) + (-1*s2) + (0*s3) + (1*s4) + (2*s5);
  if(s1 == 0 && s2 == 0 && s4 == 0 && s5 == 0 && s3 == 0) {
    error = prev_error;
  }
  return error;
}


int computePID(int error) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0; // seconds
  lastTime = now;

  if (dt <= 0) dt = 0.001; // safety

  // Integral with anti-windup
  integral += error * dt;
  integral = constrain(integral, -50, 50);

  // Derivative
  float derivative = (error - previousError) / dt;
  previousError = error;

  float output = (Kp * error) + (Ki * integral) + (Kd * derivative);

  return constrain(output, -100, 100);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);   //check after serial printing 
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);
  // if(s1 == 0 && s2 == 0 && s4 == 0 && s5 == 0 && s3 == 0) {
  //   leftSpeed = 0;
  //   rightSpeed =0;
    
  // }
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  // leftSpeed = 50;
  // rightSpeed = 50;
  // Forward


  
  analogWrite(in0, leftSpeed);
  digitalWrite(in1, LOW);
  analogWrite(in2, rightSpeed);
  digitalWrite(in3, LOW);

  // ledcWrite(0, leftSpeed);
  // ledcWrite(1, rightSpeed);
}
void leftEncoderISR() {
  leftCount++;
}

void rightEncoderISR() {
  rightCount++;
}

void turnLeft90() {
  leftCount = 0;
  rightCount = 0;

  while (leftCount < FORWARD_COUNT || rightCount < FORWARD_COUNT) {
    analogWrite(in0, 80);   // left motor PWM
    digitalWrite(in1, LOW); // reverse

    analogWrite(in2, 80);   // right motor PWM
    digitalWrite(in3, LOW); // forward
  }
  analogWrite(in0, 0);
  analogWrite(in2, 0);
  delay(50);
  leftCount = 0;
  rightCount = 0;
  // Left motor backward, right motor forward
  while (leftCount < TURN_COUNT) {
    analogWrite(in0, 80);   // left motor PWM
    digitalWrite(in1, HIGH); // reverse

    analogWrite(in2, 80);   // right motor PWM
    digitalWrite(in3, LOW); // forward
  } 

  // Stop motors
  analogWrite(in0, 0);
  analogWrite(in2, 0);

  delay(50);  // short settle delay
}

void turnRight90() {
  leftCount = 0;
  rightCount = 0;

  while (leftCount < FORWARD_COUNT || rightCount < FORWARD_COUNT) {
    analogWrite(in0, 80);   // left motor PWM
    digitalWrite(in1, LOW); // reverse

    analogWrite(in2, 80);   // right motor PWM
    digitalWrite(in3, LOW); // forward
  }
  analogWrite(in0, 0);
  analogWrite(in2, 0);
  delay(50);
  leftCount = 0;
  rightCount = 0;
  // Left motor backward, right motor forward
  while (rightCount < TURN_COUNT) {
    analogWrite(in0, 80);   // left motor PWM
    digitalWrite(in1, LOW); // reverse

    analogWrite(in2, 80);   // right motor PWM
    digitalWrite(in3, HIGH); // forward
  }

  // Stop motors
  analogWrite(in0, 0);
  analogWrite(in2, 0);

  delay(50);  // short settle delay
}

void moveForward() {
  leftCount = 0;
  rightCount = 0;

  while (leftCount < FORWARD_COUNT || rightCount < FORWARD_COUNT) {
    analogWrite(in0, 80);   // left motor PWM
    digitalWrite(in1, LOW); // reverse

    analogWrite(in2, 80);   // right motor PWM
    digitalWrite(in3, LOW); // forward
  }
  analogWrite(in0, 0);
  analogWrite(in2, 0);
  delay(50);
}

bool isJunction() {
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);

  return (s1 && s2 && s3 && s4 && s5);
}


bool isLeft() {
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);

  return (s5 && s4 && s3 && !s1 && !s2);
}
bool isRight() {
  int s1 = !digitalRead(S1);
  int s2 = !digitalRead(S2);
  int s3 = !digitalRead(S3);
  int s4 = !digitalRead(S4);
  int s5 = !digitalRead(S5);

  return  (s1 && s2 && s3 && !s4 && !s5);
}




void loop() {
  // put your main code here, to run repeatedly:
  if (isJunction()) {
    // Stop motors before turning
    analogWrite(in0, 0);
    analogWrite(in2, 0);
    delay(50);
  
    moveForward();   // or turnRight90()
    return;         // resume PID after turn
  }

   if (isLeft()) {
    // Stop motors before turning
    analogWrite(in0, 0);
    analogWrite(in2, 0);
    delay(50);
  
    turnLeft90();   // or turnRight90()
    return;         // resume PID after turn
  }

   if (isRight()) {
    // Stop motors before turning
    analogWrite(in0, 0);
    analogWrite(in2, 0);
    delay(50);
  
    turnRight90();   // or turnRight90()
    return;         // resume PID after turn
  }

  error = readError(previous_error);
  if (abs(error) < 0.5) {
    integral = 0;
  }
  previous_error = error;

  int correction = computePID(error);
  int leftMotorSpeed = baseSpeed - correction;
  int rightMotorSpeed = baseSpeed + correction;

  setMotorSpeed(leftMotorSpeed, rightMotorSpeed);



}
