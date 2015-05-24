#include <Servo.h>
double lastTime=0;
double SampleTime=100;
double lastPos = 0;
double servoMax=30;
double servoMin=-20;
double integral=0;
//pin

const int servoPin = 3;                                               //Servo Pin
int left_echo_pin = 8;
int left_trigger_pin = 7;
int right_echo_pin = 12;
int right_trigger_pin = 13;


float Kp = 3.5;                                                    //Initial Proportional Gain
float Ki = 1.0/0.1;                                                      //Initial Integral Gain
float Kd = 0;                                                    //Intitial Derivative Gain
double setpointRight, setpointLeft, pos, output;
double TOTAL_LENGTH=60;
Servo myServo;                                                       //Initialize Servo.

//states
bool reverse;
void setup() {

  Serial.begin(9600); 
  pinMode(left_echo_pin, INPUT);
  pinMode(left_trigger_pin, OUTPUT);  //Begin Serial 
  myServo.attach(servoPin);                                          //Attach Servo

  lastPos = getDistance(left_trigger_pin, left_echo_pin);                                          //Calls function readPosition() and sets the balls
  //  position as the input to the PID algorithm
  setpointLeft = 25.3;
  setpointRight=21.1;
  reverse=false;
                                //Set Output limits to -80 and 80 degrees. 
}

void loop()
{
  pos = getDistance(left_trigger_pin, left_echo_pin);
  getDistance(left_trigger_pin, left_echo_pin);
  Serial.println(setpoint-pos);
  if(abs(pos)>TOTAL_LENGTH){
  }
  else{
    output=compute(pos);                                                  //computes Output in range of -80 to 80 degrees                                           // 102 degrees is my horizontal 
    myServo.write(output+87);   
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
          double error = setpoint - pos;
           integral+= (Ki * error);
      if(integral > servoMax) integral= servoMax;
      else if(integral < servoMin) integral= servoMin;
      double dInput = (pos - lastPos);
 
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
  double rightcm = getDistance(right_trigger_pin, right_echo_pin);
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

