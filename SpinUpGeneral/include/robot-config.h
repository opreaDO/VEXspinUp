using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor_group flywheel;
extern motor intake;
extern motor roller;
extern optical opticalSensor;
extern controller Controller1;
extern digital_out indexer;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );