
//global variables
int leftServoPin = 1;  //port number for left motor
int rightServoPin = 2; //port numbersfor right motor
int right_echo_pin = 12;
int right_trigger_pin = 13;

int left_echo_pin = 8;
int left_trigger_pin = 7;
int pulse = 1500; //delay in microseconds

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
  Serial.begin(9600);
  pinMode(right_echo_pin, INPUT);
  pinMode(right_trigger_pin, OUTPUT);
  pinMode(left_echo_pin, INPUT);
  pinMode(left_trigger_pin, OUTPUT);}
  
void loop() {
  //Distance will be from left ultrasonic sensor
  double error = getError();
  double slope=error-lasterror;
  cumError=cumError+error;
  double torq = pid(error, cumError, slope, kP, kI, kD);
}

double getError(){
  distanceFromLeft=getLeftDistance();
  distanceFromRight=getRightDistance();
  distanceFromLeft=(distanceFromLeft+(TOTAL_LENGTH-distanceFromRight))/2;
  if(distanceFromLeft>TOTAL_LENGTH) distanceFromLeft=0;
  return target-distanceFromLeft;
}

double getLeftDistance(){
  double left_ping_microseconds = ping(left_trigger_pin, left_echo_pin);
  double left_distance = ping_to_cm(left_ping_microseconds);
  return left_distance;
}

double ping_to_cm(int ping_microseconds){
  double sound_cm_per_microsecond_at_sea_level = 0.034029;
  return ping_microseconds * sound_cm_per_microsecond_at_sea_level / 2;}

double getRightDistance(){
  double right_ping_microseconds = ping(right_trigger_pin, right_echo_pin);
  double right_distance = ping_to_cm(right_ping_microseconds);
}
double pid(double error, double integral, double derivative, double Kp, double Ki, double Kd) {
  return (error * Kp + integral * Ki + derivative * Kd);
}

void on(int pin){
  digitalWrite(pin, HIGH);}

void off(int pin){
  digitalWrite(pin, LOW);}

int ping(int trigger, int echo){
  int ping_time = 0;
  // turn off trigger
  off(trigger);
  delayMicroseconds(2);
  // turn on the trigger and leave it on long enough for the
  // sonar sensor to notice
  on(trigger);
  delayMicroseconds(10);
  off(trigger);
  ping_time = pulseIn(echo, HIGH);
  if(ping_time <= 0){
    ping_time = 3000;}
  // sonar needs some time to recover before pinging again,
  // so make sure it gets enough sleep right here.  50 milliseconds
  delay(50);
  return ping_time;}

