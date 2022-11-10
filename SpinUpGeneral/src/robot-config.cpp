#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT4, ratio18_1, true);
motor leftMotorB = motor(PORT3, ratio18_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT2, ratio18_1, false);
motor rightMotorB = motor(PORT1, ratio18_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 381, 304.79999999999995, mm, 1);

motor flywheelMotorA = motor(PORT15, ratio6_1, false);
motor flywheelMotorB = motor(PORT14, ratio6_1, true);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);

motor intake = motor(PORT20, ratio18_1, true);
motor roller = motor(PORT12, ratio36_1, false);

optical opticalSensor = optical(PORT11);
controller Controller1 = controller(primary);
digital_out indexer = digital_out(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

bool rollerSpinningDone = false;
bool flywheelFASSSS = false;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      // int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      // int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      // if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        // if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          // LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          // DrivetrainLNeedsToBeStopped_Controller1 = false;
        // }
      // } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        // DrivetrainLNeedsToBeStopped_Controller1 = true;
      // }
      // check if the value is inside of the deadband range
      // if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        // if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          // RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          // DrivetrainRNeedsToBeStopped_Controller1 = false;
        // }
      // } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        // DrivetrainRNeedsToBeStopped_Controller1 = true;
      // }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      // if (DrivetrainLNeedsToBeStopped_Controller1) {
        // LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        // LeftDriveSmart.spin(forward);
      // }
      // only tell the right drive motor to spin if the values are not in the deadband range
      // if (DrivetrainRNeedsToBeStopped_Controller1) {
        // RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        // RightDriveSmart.spin(forward);
      // }

      // flywheel booooolean
      if (Controller1.ButtonLeft.pressing()) {
        flywheelFASSSS = false;
      }

      if (Controller1.ButtonUp.pressing()) {
        flywheelFASSSS = true;
      }

      // flywheel controls
      if (Controller1.ButtonL2.pressing()) {
        if (flywheelFASSSS) {
          flywheel.spin(forward, 12, volt);
        }
        else if (!flywheelFASSSS) {
          flywheel.spin(forward, 8, volt);
        }
      }
      else if (Controller1.ButtonL1.pressing()) {
        flywheel.spin(forward, 0, volt);
      }

      // intake controls
      if (Controller1.ButtonR1.pressing()) {
        intake.stop();
      }
      else if (Controller1.ButtonR2.pressing()) {
        intake.spin(forward, 100, pct);
      }

      // indexer controls
      if (Controller1.ButtonX.pressing()) {
        indexer.set(true);
      }
      else {
        indexer.set(false);
      }

      // roller settings
      if ((opticalSensor.hue() <= 90) && (opticalSensor.hue() >= 0) && (opticalSensor.isNearObject())) {
        rollerSpinningDone = false;
        roller.spin(reverse, 100, pct);
      }
      else if ((opticalSensor.hue() <= 255) && (opticalSensor.hue() >= 120) && (opticalSensor.isNearObject())) {
        if (rollerSpinningDone == false) {
          roller.setVelocity(100, pct);
          roller.spinFor(reverse, 0.5, sec);
          roller.stop();
          rollerSpinningDone = true;
        }
      }
    
      //if (Controller1.ButtonY.pressing()) {
      //  roller.spin(reverse, 100, pct);
     // }
      //else if (Controller1.ButtonB.pressing()) {
      //  roller.stop();
      //}

    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}