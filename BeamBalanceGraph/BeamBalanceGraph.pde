/*
  Graph
 
 A simple example of communication from the Arduino board to the computer:
 the value of analog input 0 is sent out the serial port.  We call this "serial"
 communication becxause the connection appears to both the Arduino and the
 computer as a serial port, even though it may actually use
/* Processing code for this example
 
 // Graphing sketch
 
 
 // This program takes ASCII-encoded strings
 // from the serial port at 9600 baud and graphs them. It expects values in the
 // range 0 to 1023, followed by a newline, or newline and carriage return
 
 // Created 20 Apr 2005
 // Updated 18 Jan 2008
 // by Tom Igoe
 // This example code is in the public domain.
*/
 import processing.serial.*;
 
 Serial myPort;        // The serial port
 int xPos = 1;   // horizontal position of the graph
 float pscale = 2;
 float iscale = 10;
 float dscale = 0.2;
 float scale = 10;
 void setup () {
 // set the window size:
 size(800, 600);        
   
 // List all the available serial ports
 println(Serial.list());
 // I know that the first port in the serial list on my mac
 // is always my  Arduino, so I open Serial.list()[0].
 // Open whatever port is the one you're using.
 myPort = new Serial(this, "/dev/tty.usbmodemfd121", 9600);
 // don't generate a serialEvent() unless you get a newline character:
 myPort.bufferUntil('\n');
 // set inital background:
 background(0);
 }
 void draw () {
 // everything happens in the serialEvent()
 }
 
 void serialEvent (Serial myPort) {
 // get the ASCII string
   String inString = myPort.readStringUntil('\n');
   if (inString != null) {
     // trim off any whitespace:
     inString = trim(inString);
     String[] input = inString.split(",");
     
     // convert to an int and map to the screen height:
     float p = float(input[0]); 
     float i = float(input[1]); 
     float d = float(input[2]); 
     //p = map(p, 0, 1023, 0, height/3);
     //i = map(i, 0, 1023, 0, height/3);
     //d = map(d, 0, 1023, 0, height/3);
     
     // draw the line:
     stroke(255,0,0);
     line(xPos, height/6, xPos, height/6 - p*pscale);
     stroke(0,255,0);
     line(xPos, height/2, xPos, height/2 - i*iscale);
     stroke(0,0,255);
     line(xPos, 5*height/6, xPos, 5*height/6 - d*dscale);
     
     // at the edge of the screen, go back to the beginning:
     if (xPos >= width) {
     xPos = 0;
     background(0); 
     } 
     else {
     // increment the horizontal position:
     xPos+=3;
     }
   }
   
  
 }
