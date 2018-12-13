// Libraries ------------------------------------------------------------------------------
#include <Servo.h>
#include <Pixy2.h> //you have to download this separate
#include <SPI.h>
#include <stdio.h>
#include <SharpIR.h>

//Realtime loop Initializing-------------------------------------------------------------------
const int aliveLED = 13; //create a name for "robot alive" blinky light pin
const int eStopPin = 12; //create a name for pin connected to ESTOP switch
boolean aliveLEDState = true; //create a name for alive blinky light state to be used with timer
boolean ESTOP = true;
boolean realTimeRunStop = true; //create a name for real time control loop flag
int command = 2;
unsigned long oldLoopTime = 0; //create a name for past loop time in milliseconds
unsigned long newLoopTime = 0; //create a name for new loop time in milliseconds
unsigned long cycleTime = 0; //create a name for elapsed loop cycle time
const long controlLoopInterval = 400 ; //create a name for control loop cycle time in milliseconds


// Input definitions-------------------------------------------------------------------------
SharpIR IRLeftFront(SharpIR::GP2Y0A02YK0F, A0);
SharpIR IRLeftBack(SharpIR::GP2Y0A02YK0F, A1);
SharpIR IRRightFront(SharpIR::GP2Y0A02YK0F, A2);
SharpIR IRRightBack(SharpIR::GP2Y0A02YK0F, A3);
SharpIR IRFrontLeft(SharpIR::GP2Y0A02YK0F, A4);
SharpIR IRFrontRight(SharpIR::GP2Y0A02YK0F, A5);

//int [11][1] radialView;

// Output definitions-------------------------------------------------------------------------
#define rudderPin 3
#define propellerPin 5
#define redLedPin 22
#define blueLedPin 23
#define greenLedPin 24

// Initializing Variables ---------------------------------------------------------------------------------

int leftFrontIRMinimumDistance = 90;
int leftFrontIRMaximumDistance = 160;

int propellorSpeed = 85;
int circleRadiusValues[] = {140, 115, 100, 85, 70, 55, 20}; //TODO we probably need to remap this because of the limited movability of the servo
int circleRadius = 3; //this is straight ahead

int presenceThreshold = 150; // Threshold that basically says the ir sees something that exists
int behaviorThreshold = 100; // Threshold that decides whether the boat moves towards or away from something

int centeringThreshold = 30; // The threshold that provides a range for the centering of the iceberg using the pixy

int blocks[10];

int pixyFrameWidth = 316;  // 0 to 316 left to right
int pixyFrameHeight = 207; // 0 to 207 bottom to top

int smallIceBergDistThreshold = 30;
int iceBergAreaThreshold = 7000;

bool startCCW = true; // determines if the boat turns left or right once it is close to the iceberg

int IRLeftFrontDistCM;
int IRLeftBackDistCM;
int IRRightFrontDistCM;
int IRRightBackDistCM;
int IRFrontLeftDistCM;
int IRFrontRightDistCM;

int figure8Behavior;

// Circle variables--------------------------------------------
int tooCloseMinimumDist = 35;
int leftFrontIRMinimumDistanceC = 50;//change for new IR
int leftFrontIRMaximumDistanceC = 75;
boolean boatOutOfDock = false;
boolean firstTime = true;
unsigned long newOutDockTime;
unsigned long oldOutDockTime;


//Initializing functions
void circle();
void figure8();
void innerCircleCCW();
void goToIceBerg();

void iceBergCloseTurnLeft();
void innerCircleCW();
void iceBergCloseTurnRight();
int findIceBergX();
int findIceBergArea();

unsigned long timer;

// Initializing objects---------------------------------------------------------------------------------

Servo rudder;
Servo throttle;
Pixy2 pixy;

// Behavior states --------------------------------------------------------------------------------------
//int figure8Behavior = 0;

void setup() {
  pinMode(aliveLED, OUTPUT); // initialize aliveLED pin as an output
  pinMode(eStopPin, INPUT_PULLUP);
  Serial.begin(9600);
  rudder.attach(rudderPin);
  throttle.attach(propellerPin);
  pixy.init();
}

void loop() {

  command = getOperatorInput(); // get operator input from serial monitor
  if (command == 1) realTimeRunStop = false; // skip real time inner loop
  else realTimeRunStop = true;
  while (realTimeRunStop == true) { // if OCU-Stop not commanded, run control loop
    // Check if operator inputs a command during real-time loop eecution
    if (Serial.available() > 0) { // check to see if operator typed at OCU
      realTimeRunStop = false; // if OCU input typed, stop control loop
      command = Serial.readString().toInt(); // read command string to clear buffer
      break; // break out of real-time loop
    }
    else {
      realTimeRunStop = true; // if no operator input, run real-time loop
    }
    // Real-Time clock control. Check to see if one clock cycle has elapesed before running this controlcode
    newLoopTime = millis(); // get current Arduino time (50 days till wrap)
    if (newLoopTime - oldLoopTime >= controlLoopInterval) { // if true run flight code
      oldLoopTime = newLoopTime; // reset time stamp
      blinkAliveLED(); // toggle blinky alive light
      //pixy.ccc.getBlocks(); // grabs the blocks that the pixycam outputs

      //SENSE-sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense-------

      IRLeftFrontDistCM = IRLeftFront.getDistance();
      IRLeftBackDistCM = IRLeftBack.getDistance();
      IRRightFrontDistCM = IRRightFront.getDistance();
      IRRightBackDistCM = IRRightBack.getDistance();
      IRFrontLeftDistCM = IRFrontLeft.getDistance();
      IRFrontRightDistCM = IRFrontRight.getDistance();
      // THINK think---think---think---think---think---think---think---think---think---think---think---------

      // pick robot behavior based on operator input command typed at console
      switch (command) {
        case 1:
          Serial.println("Stop Robot");
          propellorSpeed = 85; //this should reflect motors not moving
          realTimeRunStop = false;  //maybe true
          break;
        case 2:
          Serial.println("Move straight fast ");
          propellorSpeed = 100;
          circleRadius = 3; //straight ahead
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        case 3:
          Serial.println("Circle behavior");
          propellorSpeed = 95;
          circle();
          Serial.println(IRLeftFrontDistCM);
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        case 4:
          Serial.println("Figure 8 behavior");
          propellorSpeed = 95;
          figure8();
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        default:
          Serial.println("INVALID INPUT. Robot Stopped");
          propellorSpeed = 85; //this should reflect motors not moving
          circleRadius = 3;
          boatOutOfDock = false;
          firstTime = true;
          realTimeRunStop = false;
          figure8Behavior = 0;
          break;
      }

      // ACT-act---act---act---act---act---act---act---act---act---act---act---act---act---act------------

      ESTOP = digitalRead(eStopPin); // check ESTOP switch
      setPropellorSpeed(propellorSpeed);
      setRudderAngle(circleRadius);
      // Check to see if all code ran successfully in one real-time increment
      cycleTime = millis() - newLoopTime; // calculate loop execution time
      if ( cycleTime > controlLoopInterval) {
        Serial.println("******************************************");
        Serial.println("error - real time has failed, stop robot!"); // loop took too long to run
        Serial.print(" 400 ms real-time loop took = ");
        Serial.println(cycleTime); // print loop time
        Serial.println("******************************************");
        break; // break out of real-time inner loop
      }
    } // end of "if (newLoopTime - oldLoopTime >= controlLoopInterval)" real-time loop structure
  }
}

int getOperatorInput() {
  // This function prints operator command options on the serial console and prompts
  // operator to input desired robot command
  // Serial.println(" ");

  Serial.println("==================================================================");
  Serial.println("| Robot Behavior-Commands: =, 0=(e-stops motors), 1=(robot moves forward), 2=(robot idles),  3=(robot circles), 4=(figure 8)|");
  Serial.println("| |");
  //Serial.println("================================================================");
  Serial.println("==================================================================");
  while (Serial.available() == 0) {}; // do nothing until operator input typed
  command = Serial.readString().toInt(); // read command string
  Serial.print("| New robot behavior command is: "); // give command feedback to operator
  Serial.println(command);
  Serial.println("| Type 'stop' to stop control loop and wait for new command |");
  Serial.println("==================================================================");
  return command;
}

void blinkAliveLED() {
  // This function toggles state of aliveLED blinky light LED
  // if the LED is off turn it on and vice-versa:
  if (aliveLEDState == LOW) {
    aliveLEDState = HIGH;
  } else {
    aliveLEDState = LOW;
  }
  // set the LED with the ledState of the variable:
  digitalWrite(aliveLED, aliveLEDState);
}


void setPropellorSpeed(int throttleSpeed) {
  throttle.write(throttleSpeed);
}

void setRudderAngle(int circleRad) {
  rudder.write(circleRadiusValues[circleRad]);
  Serial.println("Changing rudder angle");
}

void circle() { //this circle function is made for clockwise circles

  if (IRLeftFrontDistCM <= leftFrontIRMinimumDistanceC) {
    //change the radius of the circle
    circleRadius = 5;
  }
  else if (IRLeftFrontDistCM >= leftFrontIRMaximumDistanceC) {
    circleRadius = 3;
  }
  else if (leftFrontIRMinimumDistanceC < IRLeftFrontDistCM < leftFrontIRMaximumDistanceC) {
    circleRadius = 4;
  }
  //  }
}


void figure8() { // this function hopefully allows a continuous figure 8 to happen
  Serial.println(figure8Behavior);
  switch (figure8Behavior) {
    case 0:
      start();
      Serial.println("Start");
      break;
    case 1:
      turnRight();
      Serial.println("RightX");
      break;
    case 2:
      wallFollowCCW();
      Serial.println("CCW");
      break;
    case 3:
      turnLeft();
      Serial.println("LeftX");
      break;
    case 4:
      wallFollowCW();
      Serial.println("CW");
      break;
    default:
      propellorSpeed = 85;
      circleRadius = 3;
      Serial.println("Stop");
      break;
  }
}

void start() {
  if (boatOutOfDock == false) {
    Serial.println("boat has not left dock");
    newOutDockTime = millis();
    if (firstTime) {
      Serial.println("we are full forward!");
      firstTime = false;
      oldOutDockTime = newOutDockTime;
      propellorSpeed = 110;
      circleRadius = 3;
    }
    else if (3000 < (newOutDockTime - oldOutDockTime) && (newOutDockTime - oldOutDockTime) < 6000) {
      Serial.println("we are turning left!");
      propellorSpeed = 95;
      circleRadius = 0;
    }
    else if (6000 < (newOutDockTime - oldOutDockTime) && (newOutDockTime - oldOutDockTime) < 8500) {
      Serial.println("we are turning left!");
      propellorSpeed = 95;
      circleRadius = 3;
    }
    else if (8500 < (newOutDockTime - oldOutDockTime) && (newOutDockTime - oldOutDockTime) < 9500) {
      Serial.println("pre right!");
      propellorSpeed = 95;
      circleRadius = 6;
    }
    else if ((newOutDockTime - oldOutDockTime) > 8500) {
    Serial.println("we have left the dock and are initializing the circle");
      boatOutOfDock = true;
      firstTime = true;
      timer = millis();
    }
  }
  else {
    if (millis() - timer < 8000) {
      wallFollowCW();
    }
    else {
      figure8Behavior = 1;
      timer = millis();
    }
  }
}


void wallFollowCW() {
  if (millis() - timer < 8500) {
    if (IRLeftFrontDistCM <= tooCloseMinimumDist) {
      //change the radius of the circle
      circleRadius = 6;
    }
    else if (IRLeftFrontDistCM <= leftFrontIRMinimumDistanceC){
      circleRadius = 5;
    }
    else if (IRLeftFrontDistCM >= leftFrontIRMaximumDistanceC) {
      circleRadius = 3;
    }
    else if (leftFrontIRMinimumDistanceC < IRLeftFrontDistCM < leftFrontIRMaximumDistanceC) {
      circleRadius = 4;
    }
    else if(IRFrontLeftDistCM < 40){
      circleRadius = 5;
    }
  }
  else {
    figure8Behavior = 1;
    timer = millis();
  }
}

void wallFollowCCW() {
  if (millis() - timer < 8500) {
    if (IRRightFrontDistCM <= tooCloseMinimumDist) {
      //change the radius of the circle
      circleRadius = 0;
    }
    else if (IRRightFrontDistCM <= leftFrontIRMinimumDistanceC) {
      //change the radius of the circle
      circleRadius = 1;
    }
    else if (IRRightFrontDistCM >= leftFrontIRMaximumDistanceC) {
      circleRadius = 3;
    }
    else if (leftFrontIRMinimumDistanceC < IRRightFrontDistCM < leftFrontIRMaximumDistanceC) {
      circleRadius = 2;
    }
    else if(IRFrontRightDistCM < 20){
      circleRadius = 1;
    }
  }
  else {
    figure8Behavior = 3;
    timer = millis();
  }
}

void turnLeft() {
  if (millis() - timer < 5500) {
    circleRadius = 0; //left
  }
  else if (millis() - timer < 12500) {
    circleRadius = 3; //straight
  }
  else if (millis() - timer < 17000) {
    circleRadius = 6;//right
  }
  else if (millis() - timer > 17000) {
    figure8Behavior = 4;
    timer = millis();
  }
}

void turnRight() {
  if (millis() - timer < 5500) {  // add half second
    circleRadius = 6;//right
  }
  else if (millis() - timer < 12500) {
    circleRadius = 3; //straight
  }
  else if (millis() - timer < 17000) {
    circleRadius = 0; //left
  }
  else if (millis() - timer > 17000) {
    figure8Behavior = 2;
    timer = millis();
  }
}

// Old out of dock code
/*if (boatOutOfDock == false) {
   newOutDockTime = millis();
   Serial.println("boat has not left dock");
   if (newOutDockTime - oldOutDockTime < 500) {
     Serial.println("we are sending full forward!");
     propellorSpeed = 100;
     circleRadius = 3;
   }
   else if (1000 > (newOutDockTime - oldOutDockTime) >= 500) {
     Serial.println("we are turning left");
     oldOutDockTime = newOutDockTime;
     propellorSpeed = 95;
     circleRadius = 0;
   }
   else if ((newOutDockTime - oldOutDockTime) >= 1000) {
     Serial.println("we have left the dock and are initializing the circle");
     oldOutDockTime = newOutDockTime;
     boatOutOfDock = true;
   }
   else{
     oldOutDockTime = newOutDockTime;
   }
  }
  else {*/

// Old IR code

// irCms = (-1.813*10^-6)*(rawIR)^3 + 0.0006246*(rawIR^2)- 0.07507*(rawIR) + 3.734;



// Old figure 8 functions ----------------------------------------------------------------

//void goToIceBerg(){ // We might need some course correction code in case something happens
//  Serial.print("LeftFront: ");
//  Serial.print(IRLeftFrontDistCM);
//  Serial.print("  LeftBack: ");
//  Serial.print(IRLeftBackDistCM);
//  Serial.print("  RightFront: ");
//  Serial.print(IRRightFrontDistCM);
//  Serial.print("  RightBack: ");
//  Serial.println(IRRightBackDistCM);
//  if (millis() - timer < 2000){
//    circleRadius = 3;
//  }
//  else{
//    if(IRRightFrontDistCM < 40)
//    {
//      //circle CW
//      figure8Behavior = 1;
//      timer = millis();
//    }
//    else if(IRLeftFrontDistCM < 40)
//    {
//      //circle CCW
//      figure8Behavior = 2;
//      timer = millis();
//    }
//    else
//    {
//      circleRadius = 3; // go straight  //alter if necessary
//    }
//  }
//}
//
//void innerCircleCW(){
//   //int figure8Behavior;
//   Serial.print("LeftFront: ");
//   Serial.print(IRLeftFrontDistCM);
//   Serial.print("  LeftBack: ");
//   Serial.print(IRLeftBackDistCM);
//   Serial.print("  RightFront: ");
//   Serial.print(IRRightFrontDistCM);
//   Serial.print("  RightBack: ");
//   Serial.println(IRRightBackDistCM);
//   if(millis() - timer > 12000)
//   {
//     figure8Behavior = 0;
//     timer = millis();
//   }
//   else
//   {
//    if (IRRightFrontDistCM < smallIceBergDistThreshold){
//       circleRadius = 3; //straight
//     }
//    else if (IRRightFrontDistCM > smallIceBergDistThreshold){
//       circleRadius = 6; //turn sharp left
//     }
//   }
//}
//
//void innerCircleCCW(){
//  //int figure8Behavior;
//   Serial.print("LeftFront: ");
//   Serial.print(IRLeftFrontDistCM);
//   Serial.print("  LeftBack: ");
//   Serial.print(IRLeftBackDistCM);
//   Serial.print("  RightFront: ");
//   Serial.print(IRRightFrontDistCM);
//   Serial.print("  RightBack: ");
//   Serial.println(IRRightBackDistCM);
//   if(millis() - timer > 12000)
//   {
//     figure8Behavior = 0;
//   }
//   else
//   {
//     if (IRLeftFrontDistCM < smallIceBergDistThreshold){
//       circleRadius = 3; //straight
//     }
//     else if (IRLeftFrontDistCM > smallIceBergDistThreshold){
//       circleRadius = 0; //turn sharp left
//     }
//   }
//}
