/*Info block-----------------------------------------------------------------
- Max forward propellorSpeed = 130, Max backward propellorSpeed = 35, motor not spinning = 85
*/
// >>>>>>> 1bb4c842a8e9b565098d6790bfaee19b31983b1d

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
SharpIR IRLeftFront(SharpIR::GP2Y0A02YK0F, A0 );
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
int circleRadiusValues[] = {140,115,100,85,70,55,20}; //TODO we probably need to remap this because of the limited movability of the servo
int circleRadius = 3; //this is straight ahead

int presenceThreshold = 150; // Threshold that basically says the ir sees something that exists
int behaviorThreshold = 100; // Threshold that decides whether the boat moves towards or away from something

int centeringThreshold = 30; // The threshold that provides a range for the centering of the iceberg using the pixy

int blocks[10];

int pixyFrameWidth = 316;  // 0 to 316 left to right
int pixyFrameHeight = 207; // 0 to 207 bottom to top

int smallIceBergDistThreshold = 50;
int iceBergAreaThreshold = 7000;

bool startCCW = true; // determines if the boat turns left or right once it is close to the iceberg

int IRLeftFrontDistCM;
int IRLeftBackDistCM;
int IRRightFrontDistCM;
int IRRightBackDistCM;
int IRFrontLeftDistCM;
int IRFrontRightDistCM;

// Circle variables--------------------------------------------
int leftFrontIRMinimumDistanceC = 35;//change for new IR
int leftFrontIRMaximumDistanceC = 60;
boolean boatOutOfDock = false;
int newOutDockTime = 0;
int oldOutDockTime = 0;


//Initializing functions
void circle();
void figure8(int behavior);
void Dock();
void Catch();
int innerCircleCCW(int behavior);
int goToIceBerg(int behavior,bool CCW);

int iceBergCloseTurnLeft(int behavior);
int innerCircleCW(int behavior);
int iceBergCloseTurnRight(int behavior);
int findIceBergX();
int findIceBergArea();

//int iceBerg[3] = {0,0,0};

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

      //SENSE-sense---sense---sense---sense---sense---sense---sense---sense---sense---sense---sense-------

      IRLeftFrontDistCM = IRLeftFront.getDistance();
      IRLeftBackDistCM = IRLeftBack.getDistance();
      IRRightFrontDistCM = IRRightFront.getDistance();
      IRRightBackDistCM = IRRightBack.getDistance();
      IRFrontLeftDistCM = IRFrontLeft.getDistance();
      IRFrontRightDistCM = IRFrontRight.getDistance();
      pixy.ccc.getBlocks(); // grabs the blocks that the pixycam outputs

      // THINK think---think---think---think---think---think---think---think---think---think---think---------

      // pick robot behavior based on operator input command typed at console
      switch (command){
        case 1:
          Serial.println("Stop Robot");
          propellorSpeed=85; //this should reflect motors not moving
          realTimeRunStop = false;  //maybe true
          break;
        case 2:
          Serial.println("Move straight fast ");
          propellorSpeed=120;
          circleRadius = 3;
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //don't exit loop after running once
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
        case 5:
          Serial.println("Dock at Red Dot");
          Dock();
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        case 6:
          Serial.println("Catch Nar-hwhale");
          Catch();
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        default:
          Serial.println("INVALID INPUT. Robot Stopped");
          propellorSpeed=85; //this should reflect motors not moving
          circleRadius = 3;
          realTimeRunStop = false;
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

  // irCms = (-1.813*10^-6)*(rawIR)^3 + 0.0006246*(rawIR^2)- 0.07507*(rawIR) + 3.734;

void setPropellorSpeed(int throttleSpeed) {
  throttle.write(throttleSpeed);
}

void setRudderAngle(int circleRad){
  rudder.write(circleRadiusValues[circleRad]);
}

void circle(){ //this circle function is made for clockwise circles
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


void figure8(){ // this function hopefully allows a continuous figure 8 to happen
  int figure8Behavior; 
      Serial.println(figure8Behavior);
      switch (figure8Behavior){
        case 0:
          figure8Behavior = innerCircleCCW(figure8Behavior);
          Serial.println("CCW");
          break;
        case 1:
          figure8Behavior = goToIceBerg(figure8Behavior,startCCW);
          Serial.println("Go To");
          break;
        case 2:
          figure8Behavior = iceBergCloseTurnLeft(figure8Behavior);
          Serial.println("Left");
          break;
        case 3:
          figure8Behavior = innerCircleCW(figure8Behavior);
          Serial.println("CW");
          break;
        case 4:
          figure8Behavior = iceBergCloseTurnRight(figure8Behavior);
          Serial.println("Right");
          break;
        default:
          figure8Behavior = innerCircleCCW(figure8Behavior);
          Serial.println("CCW?");
          //goToIceBerg(figure8Behavior, startCCW);
          //Serial.println("go to?");
      } 
}
    

//figure 8 functions ----------------------------------------------------------------

int innerCircleCCW(int behavior){
  //int figure8Behavior; 
  int x = findIceBergX();
   if (pixyFrameWidth/2 - centeringThreshold <= x <= pixyFrameWidth/2 + centeringThreshold ){
     return  1;
   } else {
     if (IRLeftFrontDistCM < smallIceBergDistThreshold){ //&& IRLeftBack < 30
       circleRadius = 3; //straight
     }
     else if (IRLeftFrontDistCM > smallIceBergDistThreshold){ // && IRLeftBack < 30
       circleRadius = 0; //turn sharp left
     }
   }
}

int goToIceBerg(int behavior,bool orientation){ // We might need some course correction code in case something happens

  // if iceberg is too far to the left(on the pixy) move right
  // if iceberg is too far to the right(on the pixy) move left
  //int figure8Behavior; 
  int a = findIceBergArea();
     if (a >=  iceBergAreaThreshold){ 

          if (startCCW == true){
            startCCW = false;
            return 2;
          } else if (startCCW == false){
            startCCW = true;
            return 4;
            }

    } else {
          circleRadius = 3; // go straight  //alter if necessary
         }
  }


int iceBergCloseTurnLeft(int behavior){
    //int figure8Behavior; 
       //circle = 3; // if there's a gap between what the pixy and ir can see do this
    if (IRLeftFrontDistCM <= smallIceBergDistThreshold) {
       return 3;
       }
    else{
      circleRadius = 1;
    }
}

int innerCircleCW(int behavior){ // What is CW vs CCW?
   //int figure8Behavior; 
   int x = findIceBergX();
   if (pixyFrameWidth/2 - centeringThreshold <= x <= pixyFrameWidth/2 + centeringThreshold ){
      return 1;
   } else {
     // do the circle -- corrects for being too close or too far from inner circle
     if (IRRightFrontDistCM < 50){ //&& IRLeftBack < 30
       circleRadius = 3; //straight
     }
     else if (IRRightFrontDistCM > 50){ // && IRLeftBack < 30
       circleRadius = 6; //turn sharp left
     }
  }
}

int iceBergCloseTurnRight(int behavior){
    //int figure8Behavior; 
    if (IRLeftFrontDistCM <= smallIceBergDistThreshold) {
       return 0;
       }
    else{
      circleRadius = 5;
    }
}

void Dock() {}

void Catch(){}

/*void findIceBerg(int arr[]){
  int iceBergCenterX; 
  int iceBergCenterY; 
  int iceBergArea; 
  for (int i=0; i<pixy.ccc.numBlocks;i++){ // Grab blocks to use in the following cases
      if (pixy.ccc.blocks[i].m_signature == 1) {
         if(.5 <= pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height <= 1.5){
            iceBergCenterX = pixy.ccc.blocks[i].m_x; 
            iceBergCenterY = pixy.ccc.blocks[i].m_y; 
            iceBergArea = pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height;
            iceBerg[3] = {iceBergCenterX, iceBergCenterY, iceBergArea};
         }
      }
//  iceBerg[3] = {iceBergCenterX, iceBergCenterY, iceBergArea};
  //return iceBerg[3]; 
  }
}*/

int findIceBergX(){
  int iceBergCenterX; 
  for (int i=0; i<pixy.ccc.numBlocks;i++){ // Grab blocks to use in the following cases
      if (pixy.ccc.blocks[i].m_signature == 1) {
         if(.5 <= pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height <= 1.5){
            iceBergCenterX = pixy.ccc.blocks[i].m_x; 
         }
      }
  return iceBergCenterX;
  }
}

int findIceBergArea(){
  int iceBergArea;  
  for (int i=0; i<pixy.ccc.numBlocks;i++){ // Grab blocks to use in the following cases
      if (pixy.ccc.blocks[i].m_signature == 1) {
         if(.5 <= pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height <= 1.5){
            iceBergArea = pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height;
         }
      }
  return iceBergArea;
  }
}

/*//circle functions ---------------------------------------------
void tooClose(){
}
void tooFar(){
} */
