/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "ButtonClass.h"
#include <iostream>
#include <fstream>

using namespace vex;
using std::cout; using std::ofstream;
using std::endl; using std::string;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Controller1.Screen.clearScreen();
}

// timer settings
bool timerReset = false;
bool timerVibration = false;

// Autonomous Settings //
int desiredValue = 0;
int desiredTurnValue = 0;

// PID Values //
double kP = 0.0;         /////////////////////
double kI = 0.0;     //        Requires
double kD = 0.0;     //         Proper        
double turnkP = 0.0; //         Tuning 
double turnkD = 0.0;     /////////////////////

int error;           // sensorValue - desiredValue (Position)
int prevError = 0;   // Position 20ms ago
int derivative;      // error - prevError (Speed)

int turnError;           // sensorValue - desiredValue (Position)
int turnPrevError = 0;   // Position 20ms ago
int turnDerivative;      // error - prevError (Speed)

bool resetEncoders = false;

bool enableDrivePID = true;

int drivePID() {
  
  while(enableDrivePID) {
    
    if (resetEncoders) {
      resetEncoders = false;
      leftMotorA.resetPosition();
      leftMotorB.resetPosition();
      rightMotorA.resetPosition();
      rightMotorB.resetPosition();
    }
    
    int leftApos = leftMotorA.position(degrees);          /////////////////////////
    int leftBpos = leftMotorB.position(degrees);        //     Fetch positions   
    int rightApos = rightMotorA.position(degrees);     //    of drivetrain motors
    int rightBpos = rightMotorB.position(degrees);       /////////////////////////

    /////////////////////////////////////////////////////// Lateral Movement PID /////////////////////////////////////////////////////////////
    int avgPos = (leftApos + leftBpos + rightApos + rightBpos) / 4;  // Calculates average position of motors

    error = avgPos - desiredValue;     // Potential
    derivative = error - prevError;    // Derivative

    double lateralMotorPower = error * kP + derivative * kD;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    /////////////////////////////////////////////////////// Turning Movement PID /////////////////////////////////////////////////////////////
    int turnAvgPos = (leftApos + leftBpos + rightApos + rightBpos) / 4;  // Calculates average position of motors

    turnError = turnAvgPos - desiredTurnValue;    // Potential
    turnDerivative = turnError - turnPrevError;   // Derivative

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    leftMotorA.spin(vex::directionType::fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt); 
    leftMotorB.spin(vex::directionType::fwd, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    rightMotorA.spin(vex::directionType::fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    rightMotorB.spin(vex::directionType::fwd, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    /* "prevError" is set to "error" and the program waits 20ms before running loop, when error is fetched again, 
        meaning that "prevError" is set to the value "error" was 20ms ago. Same logic applies to "turnPrevError" and "turnError"  */
    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }




  
  return 1;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  vex::task piddy(drivePID);
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    //resets global timer to notify driver of key times 
    if (timerReset == true) {
      Brain.resetTimer();
      timerReset = true;
    }

    //when timer reaches 1:30, controller vibrates
    if ((Brain.timer(sec) > 89.5) && (Brain.timer(sec) < 90.5) && (timerVibration == false)){
      // "." = short rumble, "-" = long rumble, " " = pause
      Controller1.rumble("-");
      timerVibration = true;
    }


    //displays time left in game
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print(105 - Brain.timer(sec));
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.

  
  }
}
//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
