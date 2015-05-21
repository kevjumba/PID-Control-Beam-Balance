#include <Servo.h> 

#define sensorIR 15
Servo myservo;  // Creates a servo object
volatile float inches;
double TOTAL_LENGTH = 36;
double servoHorizontal = 87;
//global variables
double pos;
int right_echo_pin = 12;
int right_trigger_pin = 13;

int left_echo_pin = 8;
int left_trigger_pin = 7;

double distanceFromLeft=0;  //in inches
double distanceFromRight=0; //in inches

double oldRight = TOTAL_LENGTH;
double oldLeft = 0;

int acceptableError = 10;

double currentAngle;


double currentIntegral = 0;
double kP = 40;
double kI = 10;
double kD = 30; 
double target=18;
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
  currentAngle=myservo.read();

}

void loop() {
  
  //Distance will be from left ultrasonic sensor
  double error = getError();
  if(isErroneous(error)){
  }
  else{
    if(error<0){
      error*=2;
    }
    if(error>0){
      error/=2;
    }
    double slope=error-lasterror;
    cumError=cumError+error*0.02;
    double correction = pid(error, cumError, slope, kP, kI, kD);
    Serial.println("error: ");
    Serial.println(error);
    Serial.println("correction: ");
    Serial.print(correction);
    Serial.println("Servo position: ");
    Serial.println(currentAngle);
    pos = getPos(currentAngle, correction);
    myservo.write(constrain((int)(pos),73,130));
    currentAngle=myservo.read();
  }
}

double getPos(double oldPos, double correction){
  return oldPos + correction*0.02;

}

double pid(double error, double integral, double derivative, double Kp, double Ki, double Kd) {
  return (error * Kp + integral * Ki + derivative * Kd);
}

double getError(){
  distanceFromRight=getRightDistance();
  distanceFromLeft = getLeftDistance();
  Serial.print("Distance from Left: ");
  Serial.println(distanceFromLeft);
  Serial.print("Distance from Right: ");
  Serial.println(distanceFromRight);
  if(isErroneous(distanceFromRight)&&isErroneous(distanceFromLeft)) {
    return target-(oldLeft+oldRight)/2;
  }
  else if(isErroneous(distanceFromRight)&&isErroneous(distanceFromLeft)){
    oldLeft=distanceFromLeft;
    return target-distanceFromLeft;
  }
  else if(!isErroneous(distanceFromRight)&&isErroneous(distanceFromLeft)){
    oldRight=distanceFromRight;
    return target-(TOTAL_LENGTH-distanceFromRight);
  }
  oldLeft=distanceFromLeft;
  oldRight=distanceFromRight;
  
  return target-(distanceFromLeft+(TOTAL_LENGTH-distanceFromRight))/2;
}

boolean isErroneous(double number){
  return (abs(number)>TOTAL_LENGTH);
}
double getLeftDistance(){
  double left_ping_microseconds = ping(left_trigger_pin, left_echo_pin);
  double left_distance = ping_to_cm(left_ping_microseconds);
  return left_distance;
}


double ping_to_cm(int ping_microseconds){
  double sound_cm_per_microsecond_at_sea_level = 0.034029;
  return ping_microseconds * sound_cm_per_microsecond_at_sea_level / 2;
}

double getRightDistance(){
  double right_ping_microseconds = ping(right_trigger_pin, right_echo_pin);
  double right_distance = ping_to_cm(right_ping_microseconds);
  return right_distance;
}

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
    ping_time = 3000;
  }

  // sonar needs some time to recover before pinging again,
  // so make sure it gets enough sleep right here.  50 milliseconds
  delay(50);
  return ping_time;
}

void on(int pin){
  digitalWrite(pin, HIGH);
}

void off(int pin){
  digitalWrite(pin, LOW);
}


