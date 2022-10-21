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

motor flywheelMotorA = motor(PORT14, ratio6_1, false);
motor flywheelMotorB = motor(PORT15, ratio6_1, true);
motor_group flywheel = motor_group(flywheelMotorA, flywheelMotorB);

motor intake = motor(PORT20, ratio18_1, false);
motor roller = motor(PORT12, ratio36_1, false);
// motor intakeAngle = motor(PORT11, ratio18_1, false);

pneumatics poomatics = pneumatics(Brain.ThreeWirePort.D);

encoder encoderA = encoder(Brain.ThreeWirePort.A);
encoder encoderB = encoder(Brain.ThreeWirePort.B);
encoder encoderC = encoder(Brain.ThreeWirePort.C);

optical opticalSensor = optical(PORT11, false);


// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;

// flywheel speed/control values
int flywheelCurrentSpeedLevel = 1; 
double flywheelSpeedL1 = 4;
double flywheelSpeedL2 = 6;
double flywheelSpeedL3 = 8;
double flywheelSpeedL4 = 10;
double flywheelSpeedL5 = 12; 
double errorInterval = 20;
bool flywheelVibration = false;

bool flywheelRunning = false;
bool intakeRunning = false;

bool intakeAngleStopped = true;

bool drivetrainReversed = false;

bool rollerSpinningDone = false;



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
      } 
      else {
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
      } 
      else {
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
        flywheel.spin(forward, 0, rpm);
      }
      else if (Controller1.ButtonL2.pressing()) {
        if (flywheelCurrentSpeedLevel == 1){
          flywheel.spin(forward, flywheelSpeedL1, volt);
        }
        else if (flywheelCurrentSpeedLevel == 2) {
          flywheel.spin(forward, flywheelSpeedL2, volt);
        }
        else if (flywheelCurrentSpeedLevel == 3) {
          flywheel.spin(forward, flywheelSpeedL3, volt);
        }
        else if (flywheelCurrentSpeedLevel == 4) {
          flywheel.spin(forward, flywheelSpeedL4, volt);
        }
        else if (flywheelCurrentSpeedLevel == 5) {
          flywheel.spin(forward, flywheelSpeedL5, volt);
        }
      }
      

      // intake/roller controls
      if (Controller1.ButtonR1.pressing()) {
        intake.spin(reverse);
      }
      else if (Controller1.ButtonR2.pressing()) {
        if (!intakeRunning) {
          intake.spin(forward);
          intakeRunning = true;
        }
        else if (intakeRunning) {
          intake.stop();
          intakeRunning = false;
        }
      }

      // intake angle controls
      // if (Controller1.ButtonA.pressing()) {
        // intakeAngle.spin(forward);
        // intakeAngleStopped = false;
      // }
      // else if (Controller1.ButtonB.pressing()) {
        // intakeAngle.spin(reverse);
        // intakeAngleStopped = false;
      // }
      // else if (!intakeAngleStopped) {
        // intakeAngle.stop();
        // intakeAngleStopped = true;
      // }

      if (Controller1.ButtonX.pressing()) {
        poomatics.open();
        poomatics.close();
      }

      if ((Controller1.ButtonDown.pressing()) && (flywheelCurrentSpeedLevel != 1)) {
        flywheelCurrentSpeedLevel -= 1;
      }
      else if ((Controller1.ButtonRight.pressing()) && (flywheelCurrentSpeedLevel != 5)) {
        flywheelCurrentSpeedLevel += 1;
      }


      // flywheel speed adjustments + vibration
      if (flywheelCurrentSpeedLevel == 1) {
        if ((flywheel.velocity(rpm) >= flywheelSpeedL1 - errorInterval) && (flywheel.velocity(rpm) <= flywheelSpeedL1 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(rpm) <= flywheelSpeedL1 - errorInterval) && (flywheel.velocity(rpm) >= flywheelSpeedL1 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 2) {
        if ((flywheel.velocity(rpm) >= flywheelSpeedL2 - errorInterval) && (flywheel.velocity(rpm) <= flywheelSpeedL2 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(rpm) <= flywheelSpeedL2 - errorInterval) && (flywheel.velocity(rpm) >= flywheelSpeedL2 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 3) {
        if ((flywheel.velocity(rpm) >= flywheelSpeedL3 - errorInterval) && (flywheel.velocity(rpm) <= flywheelSpeedL3 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(rpm) <= flywheelSpeedL3 - errorInterval) && (flywheel.velocity(rpm) >= flywheelSpeedL3 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 4) {
        if ((flywheel.velocity(rpm) >= flywheelSpeedL4 - errorInterval) && (flywheel.velocity(rpm) <= flywheelSpeedL4 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(rpm) <= flywheelSpeedL4 - errorInterval) && (flywheel.velocity(rpm) >= flywheelSpeedL4 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      else if (flywheelCurrentSpeedLevel == 5) {
        if ((flywheel.velocity(rpm) >= flywheelSpeedL5 - errorInterval) && (flywheel.velocity(rpm) <= flywheelSpeedL5 + errorInterval) && (flywheelVibration == false))   {
          controllerFlywheelReady();
        }
        else if ((flywheel.velocity(rpm) <= flywheelSpeedL5 - errorInterval) && (flywheel.velocity(rpm) >= flywheelSpeedL5 + errorInterval) && (flywheelVibration == true))   {
          ControllerFlywheelNotReady();
        }
      }

      //display speed of flywheel on controller
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print(flywheel.velocity(rpm));

      if (Controller1.ButtonLeft.pressing()) {
        if (drivetrainReversed == false) {
          motor leftMotorA = motor(PORT2, ratio18_1, false);
          motor leftMotorB = motor(PORT1, ratio18_1, false);
          motor rightMotorA = motor(PORT4, ratio18_1, true);
          motor rightMotorB = motor(PORT3, ratio18_1, true);
          drivetrainReversed = true;
        }
        else if (drivetrainReversed == true) {
          motor leftMotorA = motor(PORT1, ratio18_1, false);
          motor leftMotorB = motor(PORT2, ratio18_1, false);
          motor rightMotorA = motor(PORT3, ratio18_1, true);
          motor rightMotorB = motor(PORT4, ratio18_1, true);
          drivetrainReversed = false;
        }
      }
    
      if ((opticalSensor.hue() <= 255) && (opticalSensor.hue() >= 120) && (opticalSensor.isNearObject())) {
        rollerSpinningDone = false;
        roller.spin(reverse);
      }
      else if ((opticalSensor.hue() <= 90) && (opticalSensor.hue() >= 0) && (opticalSensor.isNearObject())) {
        if (rollerSpinningDone == false) {
          roller.spinFor(reverse, 0.5, sec);
          roller.stop();
        }
        rollerSpinningDone = true;
      }
      

    }
  }
}   

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}