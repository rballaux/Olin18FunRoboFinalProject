// Libraries ------------------------------------------------------------------------------
#include <Servo.h>; 
#include <Pixy2.h>; 
#include <SPI.h>; 
#include <stdio.h>;


// Initializing Variables ---------------------------------------------------------------------------------
int presenceThreshold = 150; // Threshold that basically says the ir sees something that exists
int behaviorThreshold = 100; // Threshold that decides whether the boat moves towards or away from something
int blocks[10];

Servo rudder; Servo throttle; 

Pixy2 pixy;


void setup() { 
  Serial.begin(9600);  
  
  rudder.attach(1); 
  throttle.attach(2); 
  
  int ir1L = analogRead(1);  
  int ir1R = analogRead(3);
  
  pixy.init();
} 

void loop(){
    blocks = pixy.ccc.getBlocks();
    throttle.write = ; 
    decideDirectionorIceberg(ir1L, ir2L, ir1R, ir2R)
} 

void decideDirectionorIceberg() {
  if (ir1L > presenceThreshold) {// || ir2L > presenceThreshold
     leftBehavior(); 
     
  } else if (ir1R > presenceThreshold){ // || ir2R > presenceThreshold
     rightBehavior()

  } else if {pixy.ccc.block > some size {
     
  } 

void leftBehavior(int ir1L) { 
  if (ir1L > behaviorThreshold){
     rudder.write(10); // More realistic movement values needed 
  } else if (ir1L < behaviorThreshold){ 
    rudderServo.write(-10); 
  } else {} 
}


void rightBehavior(int ir1R) {  // Assuming feeding in the 
   if (ir1R > behaviorThreshold){ // tolerance + ir2R
     rudder.write(-10);           //Assuming the rudder turns should be in the opposite direction
  } else if (ir1R < behaviorThreshold){ // tolerance + ir2R
    rudder.write = 10; 
  } else {}
}
}

// This is the pixycam part and I don't know how that works yet

void pixySeeIceberg() { // Need code to make this miss the iceberg by a bit once it gets close
     if pixy == 
        throttleServo.write(); 
        rudderServo.write(pixyDirection...); 
  
}
