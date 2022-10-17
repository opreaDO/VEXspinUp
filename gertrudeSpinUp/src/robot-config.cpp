#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;
controller Controller1 = controller(primary);

// VEXcode device constructors
motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio18_1, true);
motor rightMotorB = motor(PORT4, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);

motor flywheelMotorA = motor(PORT6, ratio18_1, false);
motor flywheelMotorB = motor(PORT7, ratio18_1, true);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);

motor intakeRoller = motor(PORT9, ratio18_1, false);
motor intakeAngle = motor(PORT11, ratio18_1, false);

encoder encoderA = encoder(Brain.ThreeWirePort.A);
encoder encoderB = encoder(Brain.ThreeWirePort.B);
encoder encoderC = encoder(Brain.ThreeWirePort.C);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// flywheel speed/control values
int flywheelCurrentSpeedLevel = 1; 
double flywheelSpeedL1 = 25.0;
double flywheelSpeedL2 = 45.0;
double flywheelSpeedL3 = 60.0;
double flywheelSpeedL4 = 80.0;
double flywheelSpeedL5 = 100.0; 
double errorInterval = 2;
bool flywheelVibration = false;

bool intakeAngleStopped = true;

bool timerReset = false;
bool timerVibration = false;

void controllerFlywheelReady() {
  flywheelVibration = true;
  Controller1.Screen.setCursor(1, 2);
  Controller1.Screen.print("Ready to Shoot");
  Controller1.rumble(".");
}

void ControllerFlywheelNotReady() {
  flywheelVibration = false;
  Controller1.Screen.setCursor(1, 2);
  Controller1.Screen.clearLine();
}

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {

      //resets global timer to notify driver of key times 
      if (timerReset == true) {
        Brain.resetTimer();
        timerReset = true;
      }

      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3
      // right = Axis2
      int drivetrainLeftSideSpeed = Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis2.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }

      // flywheel controls
      if (Controller1.ButtonL1.pressing()) {
        flywheel.spin(forward);
      }
      else if (Controller1.ButtonL2.pressing()) {
        flywheel.stop();
      }

      // intake/roller controls
      if (Controller1.ButtonR1.pressing()) {
        intakeRoller.spin(forward);

      }
      else if (Controller1.ButtonR2.pressing()) {
        intakeRoller.stop();
      }

      // intake angle controls
      if (Controller1.ButtonA.pressing()) {
        intakeAngle.spin(forward);
        intakeAngleStopped = false;
      }
      else if (Controller1.ButtonB.pressing()) {
        intakeAngle.spin(reverse);
        intakeAngleStopped = false;
      }
      else if (!intakeAngleStopped) {
        intakeAngle.stop();
        intakeAngleStopped = true;
      }

      if ((Controller1.ButtonDown.pressing()) && (flywheelCurrentSpeedLevel != 1)) {
        flywheelCurrentSpeedLevel -= 1;
      }
      else if ((Controller1.ButtonUp.pressing()) && (flywheelCurrentSpeedLevel != 5)) {
        flywheelCurrentSpeedLevel += 1;
      }


      // flywheel speed adjustments + vibration
      if (flywheelCurrentSpeedLevel == 1) {
        flywheel.setVelocity(flywheelSpeedL1, pct);
        if ((flywheel.velocity(pct) >= flywheelSpeedL1 - errorInterval) && (flywheel.velocity(pct) <= flywheelSpeedL1 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(pct) <= flywheelSpeedL1 - errorInterval) && (flywheel.velocity(pct) >= flywheelSpeedL1 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 2) {
        flywheel.setVelocity(flywheelSpeedL2, pct);
        if ((flywheel.velocity(pct) >= flywheelSpeedL2 - errorInterval) && (flywheel.velocity(pct) <= flywheelSpeedL2 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(pct) <= flywheelSpeedL2 - errorInterval) && (flywheel.velocity(pct) >= flywheelSpeedL2 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 3) {
        flywheel.setVelocity(flywheelSpeedL3, pct);
        if ((flywheel.velocity(pct) >= flywheelSpeedL3 - errorInterval) && (flywheel.velocity(pct) <= flywheelSpeedL3 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(pct) <= flywheelSpeedL3 - errorInterval) && (flywheel.velocity(pct) >= flywheelSpeedL3 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 4) {
        flywheel.setVelocity(flywheelSpeedL4, pct);
        if ((flywheel.velocity(pct) >= flywheelSpeedL4 - errorInterval) && (flywheel.velocity(pct) <= flywheelSpeedL4 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(pct) <= flywheelSpeedL4 - errorInterval) && (flywheel.velocity(pct) >= flywheelSpeedL4 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 5) {
        flywheel.setVelocity(flywheelSpeedL5, pct);
        if ((flywheel.velocity(pct) >= flywheelSpeedL5 - errorInterval) && (flywheel.velocity(pct) <= flywheelSpeedL5 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(pct) <= flywheelSpeedL5 - errorInterval) && (flywheel.velocity(pct) >= flywheelSpeedL5 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      //display speed of flywheel on controller
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print(Brain.timer(sec));

      //when timer reaches 1:30, controller vibrates
      if ((Brain.timer(sec) > 89.5) && (Brain.timer(sec) < 90.5) && (timerVibration == false)){
        // "." = short rumble, "-" = long rumble, " " = pause
        Controller1.rumble("-");
        timerVibration = true;
      }


      //displays time left in game
      Controller1.Screen.setCursor(3, 1);
      Controller1.Screen.print(105 - Brain.timer(sec));


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