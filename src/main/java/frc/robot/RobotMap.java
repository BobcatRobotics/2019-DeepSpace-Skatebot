package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // Drive  Train
	public static int driveRightMotorFront = 2;
	public static int driveRightMotorMiddle = 23;
	public static int driveRightMotorRear = 24;
	public static int driveLeftMotorFront = 1;
	public static int driveLeftMotorMiddle = 21;
	public static int driveLeftMotorRear = 22;
	
	// Drive Train Encoders
	public static int leftEncoderChannel1 = 2;
	public static int leftEncoderChannel2 = 3;
	public static int rightEncoderChannel1 = 0;
	public static int rightEncoderChannel2 = 1;

 	// Joy Sticks
	public static int leftJoystick = 0;
	public static int rightJoystick = 1;
	public static int gamePad = 2;

}