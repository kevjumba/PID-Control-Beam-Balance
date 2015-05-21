#include <Servo.h> 

#define sensorIR 15
Servo myservo;  // Creates a servo object
double pos = 95;    // Variable to store the servos angle 
volatile float inches;
//global variables

int right_echo_pin = 12;
int right_trigger_pin = 13;

int left_echo_pin = 8;
int left_trigger_pin = 7;
int pulse = 1500; //delay in microseconds

double distanceFromLeft=0;  //in inches
double distanceFromRight=0; //in inches

double oldRight = 0;
double oldLeft = 0;

double TOTAL_LENGTH = 12;

double currentIntegral = 0;
double kP = 0.07;
double kI = 0;
double kD = 0; 
double target=8;
double lasterror=0;
double cumError=0;
//i2c slave address AD0 pin 9 at b1101000 and b1101001
void setup(){
  Serial.begin(9600);
  pinMode(right_echo_pin, INPUT);
  pinMode(right_trigger_pin, OUTPUT);
  pinMode(left_echo_pin, INPUT);
  pinMode(left_trigger_pin, OUTPUT);
  myservo.attach(3);  // Assigns data pin to your servo object, must be digital port

}
  
void loop() {
  //Distance will be from left ultrasonic sensor
  double error = getError();
  double slope=error-lasterror;
  cumError=cumError+error;
  double correction = pid(error, cumError, slope, kP, kI, kD);
  Serial.print("correction: ");
  Serial.println(correction);
  Serial.print("pos: ");
  Serial.println(pos);
  pos = getPos(pos, correction);
  myservo.write(constrain((int)(pos+0.5),73,110));
}

double getPos(double oldPos, double correction){
 return oldPos + correction;
  
}

double getError(){
  distanceFromRight=getRightDistance();
  distanceFromLeft = getLeftDistance();
  distanceFromLeft=(distanceFromLeft+(TOTAL_LENGTH-distanceFromRight))/2;
  Serial.print("distanceFromLeft: ");
  Serial.println(distanceFromLeft);
  Serial.print("distanceFromRight: ");
  Serial.println(distanceFromRight);
  if(distanceFromLeft > TOTAL_LENGTH) distanceFromLeft=0;
  return target-distanceFromLeft;
}

double getLeftDistance(){
  double left_ping_microseconds = ping(left_trigger_pin, left_echo_pin);
  double left_distance = ping_to_cm(left_ping_microseconds);
  if(abs(oldLeft - left_distance) > 10){
   return oldLeft; 
  }
  oldLeft = left_distance;
  return left_distance;
}

double ping_to_cm(int ping_microseconds){
  double sound_cm_per_microsecond_at_sea_level = 0.034029;
  return ping_microseconds * sound_cm_per_microsecond_at_sea_level / 2;}

double getRightDistance(){
  double right_ping_microseconds = ping(right_trigger_pin, right_echo_pin);
  double right_distance = ping_to_cm(right_ping_microseconds);
  if(abs(oldRight - right_distance) > 10){
   return oldRight; 
  }
  oldRight = right_distance;
  return right_distance - 11;
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
