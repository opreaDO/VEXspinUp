using namespace vex;

extern brain Brain;
extern controller Controller1;

// VEXcode devices
extern motor leftMotorA;
extern motor leftMotorB;
extern motor_group LeftDriveSmart;
extern motor rightMotorA;
extern motor rightMotorB;
extern motor_group RightDriveSmart;
extern drivetrain Drivetrain;

extern motor flywheelMotorA;
extern motor flywheelMotorB;
extern motor_group flywheel;

extern motor intakeRoller;
extern motor intakeAngle;

extern pneumatics poomatics;

extern encoder encoderA;
extern encoder encoderB;
extern encoder encoderC;


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );