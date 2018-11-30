// Libraries ------------------------------------------------------------------------------
#include <Servo.h>;
//#include <Pixy2.h>; //you have to download this separate
#include <SPI.h>;
#include <stdio.h>;

// input definitions-------------------------------------------------------------------------
#define IRPinLeftFront 0
#define IRPinLeftBack 1
#define IRPinRightFront 2
#define IRPinRightBack 3

// Output definitions-------------------------------------------------------------------------
#define rudderPin 3
#define propellerPin 5
#define redLedPin 22
#define blueLedPin 23
#define greenLedPin 24

// Initializing Variables ---------------------------------------------------------------------------------
int rawIRLeftFront;
int rawIRLeftBack;
int rawIRRightFront;
int rawIRRightBack;

const int aliveLED = 13; //create a name for "robot alive" blinky light pin
const int eStopPin = 12; //create a name for pin connected to ESTOP switch
boolean aliveLEDState = true; //create a name for alive blinky light state to be used with timer
boolean ESTOP = true;
boolean realTimeRunStop = true; //create a name for real time control loop flag
int command = 0;
unsigned long oldLoopTime = 0; //create a name for past loop time in milliseconds
unsigned long newLoopTime = 0; //create a name for new loop time in milliseconds
unsigned long cycleTime = 0; //create a name for elapsed loop cycle time
const long controlLoopInterval
  = 1000 ; //create a name for control loop cycle time in milliseconds

int leftFrontIRDistance = 100;
int rightFrontIRDistance = 100;

int presenceThreshold = 150; // Threshold that basically says the ir sees something that exists
int behaviorThreshold = 100; // Threshold that decides whether the boat moves towards or away from something
int blocks[10];

Servo rudder;
Servo throttle;

//Pixy2 pixy;

void setup() {
  pinMode(aliveLED, OUTPUT); // initialize aliveLED pin as an output
  pinMode(eStopPin, INPUT_PULLUP);
  Serial.begin(9600);
  rudder.attach(rudderPin);
  throttle.attach(propellerPin);
  //  pixy.init();
  readAllBoatInput();
}

void loop() {
  //blocks = pixy.ccc.getBlocks();
  command = getOperatorInput(); // get operator input from serial monitor
  if (command == 0) realTimeRunStop = false; // skip real time inner loop
  else realTimeRunStop = true;
  //decideDirectionorIceberg(stateInt);
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
      readAllBoatInput();
      // THINK think---think---think---think---think---think---think---think---think---think---think---------

      // pick robot behavior based on operator input command typed at console
      if ( command == 0) {
        Serial.println("Stop Robot");
        realTimeRunStop = false; //exit real time control loop
        setPropellorSpeed(90);
        break;
      }
      else if (command == 1 ) { //Move robot to Operator commanded position
        Serial.println("Move robot ");
        setPropellorSpeed(120);
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true; //don't exit loop after running once
      }
      else if (command == 2) { //Make robot alive with small motions
        Serial.println("Idle Robot");
        Serial.println("Type stop to stop robot");
        realTimeRunStop = true; //run loop continually
      }
      else
      {
        Serial.println("***** WARNING *******Invalid Input, Robot Stopped, Please try again!");
        realTimeRunStop = false;
      }
      // ACT-act---act---act---act---act---act---act---act---act---act---act---act---act---act------------
      ESTOP = digitalRead(eStopPin); // check ESTOP switch
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

void decideDirectionorIceberg(int stateInt) {
  if (stateInt == 0) { //this is the circling state
    if (rawIRLeftFront > presenceThreshold) {// || ir2L > presenceThreshold
      leftBehavior();

    } else if (rawIRRightFront > presenceThreshold) { // || ir2R > presenceThreshold
      rightBehavior();

    }
  }
}

void leftBehavior() {
  if (rawIRLeftFront > behaviorThreshold) {
    rudder.write(10); // More realistic movement values needed
  } else if (rawIRLeftFront < behaviorThreshold) {
    rudder.write(0);
  } else {}
}
void rightBehavior() {  // Assuming feeding in the
  if (rawIRRightFront > behaviorThreshold) { // tolerance + ir2R
    rudder.write(10);           //Assuming the rudder turns should be in the opposite direction
  } else if (rawIRRightFront < behaviorThreshold) { // tolerance + ir2R
    rudder.write(0);
  } else {}
}

void readAllBoatInput() {
  rawIRLeftFront = analogRead(IRPinLeftFront);
  rawIRLeftBack = analogRead(IRPinLeftBack);
  rawIRRightFront = analogRead(IRPinRightFront);
  rawIRRightBack = analogRead(IRPinRightBack);
}
int getOperatorInput() {
  // This function prints operator command options on the serial console and prompts
  // operator to input desired robot command
  // Serial.println(" ");

  Serial.println("==================================================================");
  Serial.println("| Robot Behavior-Commands: =, 0=(e-stops motors), 2=(robot idles)|");
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

void setPropellorSpeed(int motorSpeed) {
  throttle.write(motorSpeed);
}
