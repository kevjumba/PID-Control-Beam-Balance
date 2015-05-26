#include <Servo.h>
double lastTime=0;
double SampleTime=100;
double lastPos = 0, lastoutput=0;
double servoMax=30;
double servoMin=-20;
double integral=0, derivative=0;
Servo myServo;
//pin

const int servoPin = 3;                                               //Servo Pin
int left_echo_pin = 8;
int left_trigger_pin = 7;
int right_echo_pin = 12;
int right_trigger_pin = 13;
int horizontal = 88;


float Kp = 2.0;                                                    //Initial Proportional Gain
float Ki = 0.1*0.1;                                                      //Initial Integral Gain
float Kd = 1.5/0.1;                                                    //Intitial Derivative Gain
double setpointRight, setpointLeft, pos, output;
bool reverse;
bool over;

double TOTAL_LENGTH=60;
void setup(){
  Serial.begin(9600);
  myServo.attach(servoPin);
  
  
  pinMode(right_echo_pin, INPUT);
  pinMode(right_trigger_pin, OUTPUT);
  pinMode(left_echo_pin, INPUT);
  pinMode(left_trigger_pin, OUTPUT);
  
  lastPos = getDistance(left_trigger_pin, left_echo_pin);
  reverse=false;
  over = false;
  setpointLeft=18;
  setpointRight=18;
}
void loop()
{
  double error;
  pos = getDistance(left_trigger_pin, left_echo_pin);
  pos = readPosition();
  getDistance(left_trigger_pin, left_echo_pin); //used to delay loop
  if(reverse) {
    error=(-(setpointRight-pos));
   // Serial.println(error);
    //Serial.println(derivative);
    //Serial.println("");
   }
  else  {   
    error = setpointLeft-pos;
    // Serial.println(error);
     //Serial.println(derivative);
     //Serial.println("");
  
   }
  if(abs(pos)<=TOTAL_LENGTH) {
    output=compute(pos);
    lastoutput=horizontal+pos;
    if(abs(error)< 2.0 && abs(derivative) < 0.2) { //Checks if ball is close to center and moving very slowly
      over = true;
    }
    if(over && abs(error) < 3) { //Checks to see if ball has rolled away from the center
      myServo.write(horizontal);
    }
    else myServo.write(output+horizontal);  
  }
                                       //Writes value of Output to servo


}

double compute(double pos)
{
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      /*Compute all the working error variables*/
      double error;
      if(reverse)
          error = setpointRight - pos;
      else
          error = setpointLeft - pos;
           integral+= (Ki * error);
      if(integral > servoMax) integral= servoMax;
      else if(integral < servoMin) integral= servoMin;
      double dInput = (pos - lastPos);
      derivative = dInput;
      /*Compute PID Output*/
      double output = Kp * error + integral- Kd * dInput;
      
	  if(output > servoMax) output = servoMax;
      else if(output < servoMin) output = servoMin;
	  
      /*Remember some variables for next time*/
      lastPos = pos;
      lastTime = now;
      if(reverse) return -output;
      return output;
   }
   else return 0;
}



double getDistance(int trigger, int echo){
  double ping_microseconds = ping(trigger, echo);
  double distance = ping_to_cm(ping_microseconds);
  return distance;
}


double ping_to_cm(int ping_microseconds){
  double sound_cm_per_microsecond_at_sea_level = 0.034029;
  return ping_microseconds * sound_cm_per_microsecond_at_sea_level / 2;
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

double readPosition() {                                                    //Don't set too low or echos will run into eachother.      
  double leftcm=getDistance(left_trigger_pin, left_echo_pin);
  Serial.println(leftcm);
  double rightcm = getDistance(right_trigger_pin, right_echo_pin);
  Serial.println(rightcm);
  Serial.println("");
  if(isErroneous(rightcm)&&isErroneous(leftcm)) {
    // Serial.print("Both error flag: ");
    return TOTAL_LENGTH+1;
  }
  else if(isErroneous(rightcm)&&!isErroneous(leftcm)){
    //Serial.print("Right error flag: ");
    reverse=false;
    return leftcm;
  }
  else if(!isErroneous(rightcm)&&isErroneous(leftcm)){
    //Serial.print("Left error flag: ");
    reverse=true;
    return rightcm;
  }
  reverse=false;
  return leftcm;
    //Returns distance value.
}

boolean isErroneous(double number){
  return (abs(number)>TOTAL_LENGTH);
}

void on(int pin){
  digitalWrite(pin, HIGH);
}

void off(int pin){
  digitalWrite(pin, LOW);
}

