#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

motor leftMotorA = motor(PORT1, ratio18_1, false);
motor leftMotorB = motor(PORT2, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT3, ratio18_1, true);
motor rightMotorB = motor(PORT4, ratio18_1, true);
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

int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {

    if (Controller1.ButtonX.pressing()) {
      indexer.set(true);
    }
    else {
      indexer.set(false);
    }
    
  }
}   

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}