/*Info block-----------------------------------------------------------------
- Max forward propellorSpeed = 130, Max backward propellorSpeed = 35, motor not spinning = 85
*/

// Libraries ------------------------------------------------------------------------------
#include <Servo.h>
#include <Pixy2.h> //you have to download this separate
#include <SPI.h>
#include <stdio.h>

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
const long controlLoopInterval = 1000 ; //create a name for control loop cycle time in milliseconds


// Input definitions-------------------------------------------------------------------------
#define IRPinLeftFront A2
#define IRPinLeftBack A1
#define IRPinRightFront A3
#define IRPinRightBack A3
#define IRPinFrontLeft A4
#define IRPinFrontRight A5

int [11][1] radialView;

// Output definitions-------------------------------------------------------------------------
#define rudderPin 3
#define propellerPin 5
#define redLedPin 22
#define blueLedPin 23
#define greenLedPin 24

// Initializing Variables ---------------------------------------------------------------------------------
int IRLeftFront;
int IRLeftBack;
int IRRightFront;
int IRRightBack;
int IRFrontLeft;
int IRFrontRight;

int leftFrontIRMinimumDistance = 400;
int leftFrontIRMaximumDistance = 500;

int propellorSpeed = 85;
int circleRadiusValues[] = {140,115,100,85,70,55,20}; //TODO we probably need to remap this because of the limited movability of the servo
int circleRadius = 3; //this is straight ahead

int presenceThreshold = 150; // Threshold that basically says the ir sees something that exists
int behaviorThreshold = 100; // Threshold that decides whether the boat moves towards or away from something

int centeringThreshold = 30; // The threshold that provides a range for the centering of the iceberg using the pixy

int blocks[10];

int pixyFrameWidth = 316;  // 0 to 316 left to right
int pixyFrameHeight = 207; // 0 to 207 bottom to top

int irInches;

int lastCase = 0; // determines if the boat turns left or right once it is close to the iceberg

// Initializing objects---------------------------------------------------------------------------------

Servo rudder;
Servo throttle;
Pixy2 pixy;

// Behavior states --------------------------------------------------------------------------------------
int figure8Behavior = 0;

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

      IRLeftFront = convertRawIRToInches(analogRead(IRPinLeftFront));
      IRLeftBack = convertRawIRToInches(analogRead(IRPinLeftBack));
      IRRightFront = convertRawIRToInches(analogRead(IRPinRightFront));
      IRRightBack = convertRawIRToInches(analogRead(IRPinRightBack));
      IRFrontLeft = convertRawIRToInches(analogRead(IRPinFrontLeft));
      IRFrontRight = convertRawIRToInches(analogRead(IRPinFrontRight));
      int i; // Initializes the i variable used by the pixycam
      pixy.ccc.getBlocks() // grabs the blocks that the pixycam outputs


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
          propellorSpeed = 100;
          circle();
          Serial.println("Type 0 to stop robot");
          realTimeRunStop = true; //run loop continually
          break;
        case 4:
          Serial.println("Figure 8 behavior");
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
        Serial.print(" 1000 ms real-time loop took = ");
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

int convertRawIRToInches(int rawIR){
  //TODO change this calculation
    return (-1.813*10^-6)*(rawIR)^3 + 0.0006246*(rawIR^2)- 0.07507*(rawIR) + 3.734;

}

void setPropellorSpeed(int throttleSpeed) {
  throttle.write(throttleSpeed);
}

void setRudderAngle(int circleRad){
  rudder.write(circleRadiusValues[circleRad]);
}

void circle(){ //this circle function is made for clockwise circles
  if (IRLeftFront <= leftFrontIRMinimumDistance) {
    //change the radius of the circle
    circleRadius = 3;
  }
  else if (IRLeftFront >= leftFrontIRMaximumDistance){
    circleRadius = 1;
  }
  else if (leftFrontIRMinimumDistance < IRLeftFront < leftFrontIRMaximumDistance){
    circleRadius = 2;
  }
}


void figure8(){ // this function hopefully allows a continuous figure 8 to happen
      switch (figure8Behavior){
        case 0 :
          innerCircleCCW();
          // if iceberg is visible in the middle of the screen change figure8Behavior to 1 the radius of the circle
          break;
        case 1:
          goToIceBerg();
          // determines left or right
          // read lastCase here. Once iceberg is big enough check If = 0 go to case 2 and set lastCase = 1. If = 1 go to case 4 and set lastCase to 0
          break;
        case 2:
          iceBergCloseTurnLeft();
          // once the iceberg is out of view go to case 3
          break;
        case 3:
          innerCircleCW();
          // once the iceberg is visible in the middle of the screen go back to case 1
          break;
        case 4:
          iceBergCloseTurnRight();
          // once the iceberg is out of view go to case 0
        default:
         // this may or may not be helpful
          goToIceBerg();
      }
  }


//figure 8 functions ----------------------------------------------------------------

void innerCircleCCW(){
  //look in the radialView for left side
  // if too close go further away
  // if too far go closer
  //change the radius of the circle

   if (pixy.ccc.blocks[i].m_signature == 1 && pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height == icebergRatio
     && pixyFrameWidth/2 - centeringThreshold <= pixy.ccc.blocks[i].m_x <= pixyFrameWidth/2 + centeringThreshold ){
     figure8Behavior = 1;
   } else {
     // do the circle
   }

}
    //uses tooClose() & tooFar()

void goToIceBerg(){ // We might need some course correction code in case something happens

  // if iceberg is too far to the left(on the pixy) move right
  // if iceberg is too far to the right(on the pixy) move left
  // if pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height >= icebergVisibleArea change figure8Behavior to 2

     if (pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height >=  icebergAreaThreshold && pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height == icebergRatio
         && pixy.ccc.blocks[i].m_signature == 1){ // there should be some sort of threshold for ratio (+ or - this amount)

          if (lastCase == 0){
            lastCase = 1;
            figure8Behavior = 2;
          } else if (lastCase == 1){
            lastCase = 0;
            figure8Behavior = 4;}

    } else {
          circle = 3; // go straight
         }
  }
}

void iceBergCloseTurnLeft(){
      //if {pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height >= icebergCloseArea) && pixy.ccc.blocks[i].m_signature == 1{ // 316 is frame size. The .25 is arbitrary.

    circleRadiusValues[115]; // I assume this is left
       //circle = 3; // if there's a gap between what the pixy and ir can see do this
    if (irLeftFront <= presenceThreshold) {
       figure8Behavior = 3;
       }
}

void innerCircleCW(){ // What is CW vs CCW?

 if (pixy.ccc.blocks[i].m_signature == 1 && pixy.ccc.blocks[i].m_width/pixy.ccc.blocks[i].m_height == icebergRatio
     && pixyFrameWidth/2 - centeringThreshold <= pixy.ccc.blocks[i].m_x <= pixyFrameWidth/2 + centeringThreshold ){
     figure8Behavior = 1;

 } else {
    // do the circle -- corrects for being too close or too far from inner circle
 }
}

//goToIceBerg

void iceBergCloseTurnRight(){
    //if {pixy.ccc.blocks[i].m_width*pixy.ccc.blocks[i].m_height >= icebergCloseArea) && pixy.ccc.blocks[i].m_signature == 1{ // 316 is frame size. The .25 is arbitrary.

    circleRadiusValues[55]; // I assume this is right
    //circle = 3; // if there's a gap between what the pixy and ir can see do this
    if (irLeftFront <= presenceThreshold) {
       figure8Behavior = 0;
       }
}



/*//circle functions ---------------------------------------------
void tooClose(){

}
void tooFar(){

} */
