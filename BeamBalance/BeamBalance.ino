
//global variables
int leftServoPin = 1;  //port number for left motor
int rightServoPin = 2; //port numbersfor right motor
int pulse = 1500; //delay in microseconds

int right_motor_speed_pin = 3;
int right_motor_forward_pin = 4;
int right_motor_backward_pin = 5;
 
int left_motor_speed_pin = 8;
int left_motor_forward_pin = 9;
int left_motor_backward_pin = 10;

double distanceFromLeft=0;  //in inches
double distanceFromRight=0; //in inches
double TOTAL_LENGTH = 12;

double currentIntegral = 0;
double kP = 0.2;
double kI = 0.3;
double kD = 0; 
double target=6;
double lasterror=0;
double cumError=0;
//i2c slave address AD0 pin 9 at b1101000 and b1101001
void setup(){
  Wire.begin();
  pinMode(leftServoPin, OUTPUT);
  pinMode(rightServoPin, OUTPUT);
  
  pinMode(right_motor_speed_pin, OUTPUT);
  pinMode(right_motor_forward_pin, OUTPUT);
  pinMode(right_motor_backward_pin, OUTPUT);
  pinMode(left_motor_speed_pin, OUTPUT);
  pinMode(left_motor_forward_pin, OUTPUT);
  pinMode(left_motor_backward_pin, OUTPUT);
  
  //set all motors to off by default
  off(left_motor_speed_pin);
  off(left_motor_forward_pin);
  off(left_motor_backward_pin);
  off(right_motor_speed_pin);
  off(right_motor_forward_pin);
  off(right_motor_backward_pin);
  Serial.begin(9600);
}
void loop() {
  //Distance will be from left ultrasonic sensor
  error = getError();
  double slope=error-lasterror;
  cumError=cumError+error;
  double torq = pid(error, cumError, slope, kP, kI, kD);

  go(torq, torq, 50);
}

double error(){
  distanceFromLeft=getLeftDistance();
  distanceFromRight=getRightDistance();
  distanceFromLeft=(distanceFromLeft+(TOTAL_LENGTH-distanceFromRight))/2;
  if(distanceFromleft>TOTAL_LENGTH) distanceFromLeft=0;
  return target-distanceFromLeft;
}

double pid(double error, double integral, double derivative, double Kp, double Ki, double Kd) {
  return (error * Kp + integral * Ki + derivative * Kd);
}

void on(int pin){
  digitalWrite(pin, HIGH);
}
 
void off(int pin){
  digitalWrite(pin, LOW);
}

void go(int left_motor_speed, int right_motor_speed, int time){
  set_motor(left_motor_speed_pin, left_motor_forward_pin, left_motor_backward_pin, left_motor_speed);
  set_motor(right_motor_speed_pin, right_motor_forward_pin, right_motor_backward_pin, right_motor_speed);
  delay(time);
  set_motor(left_motor_speed_pin, left_motor_forward_pin, left_motor_backward_pin, 0);
  set_motor(right_motor_speed_pin, right_motor_forward_pin, right_motor_backward_pin, 0);
}
 

void set_motor(int speed_pin, int forward_pin, int backward_pin, int speed){
  if(speed > 0){
    off(backward_pin);
    on(forward_pin);
  }else if(speed < 0){
    off(forward_pin);
    on(backward_pin);
    speed = -speed;
  }else{ // speed is 0
    off(forward_pin);
    off(backward_pin);
  }if(speed > 255){
    speed = 255;
  }
  analogWrite(speed_pin, speed);
}
