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

using namespace vex;

// A global instance of competition
competition Competition;

bool dori = false;

int autonProcess() {
  Drivetrain.drive(reverse);
  roller.spinFor(reverse, 0.7, sec);
  vex::task::sleep(700);
  Drivetrain.stop();
  flywheel.spin(forward, 12, volt);
  vex::task::sleep(2500);
  indexer.set(true);

  return 1;
}


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

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
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
  vex::task TBH_(autonProcess);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------xxsx------------------------------------------------------*/

void usercontrol(void) {
  // closing indexer after auton
  indexer.set(false);
  // User control code here, inside the loop
  while (1) {
    if (Controller1.ButtonY.pressing()) {
      dori = true;
    }
    else if (Controller1.ButtonB.pressing()) {
      dori = false;
    }
    
    if (dori == false) {
      leftMotorA.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);                 
      leftMotorB.spin(vex::directionType::fwd, Controller1.Axis3.value(), vex::velocityUnits::pct);                     
      rightMotorA.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);                 
      rightMotorB.spin(vex::directionType::fwd, Controller1.Axis2.value(), vex::velocityUnits::pct);                  
    }
    else if (dori == true) {
      leftMotorA.spin(vex::directionType::fwd, -(Controller1.Axis2.value()), vex::velocityUnits::pct);                 
      leftMotorB.spin(vex::directionType::fwd, -(Controller1.Axis2.value()), vex::velocityUnits::pct);                       
      rightMotorA.spin(vex::directionType::fwd, -(Controller1.Axis3.value()), vex::velocityUnits::pct);                 
      rightMotorB.spin(vex::directionType::fwd, -(Controller1.Axis3.value()), vex::velocityUnits::pct);                  
    }
    
  
    wait(10, msec); // Sleep the task for a short amount of time to
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
