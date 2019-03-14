/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.commands;

import frc.robot.OI;
import frc.robot.lib.RioLogger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TargetSkateBot extends Command {
	// These following constants are used in to compute distance to Target
	private static double TARGET_HEIGHT = 50.0; // height of target from floor (inches)
	private static double CAMERA_HEIGHT = 8.0;  // height of camera from floor (inches)
	private static double CAMERA_ANGLE = 0.0;   // angle of camera from floor (degrees)
	private static double DISTANCE_TO_SLOW_DOWN = 24.0; // (inches)
	private static double MAX_SPEED = 0.7; // Drive Motor Limit. Used to cap the speed of the bot

	private boolean hasValidTarget = false;
	private double driveCommand = 0.0;
	private double steerCommand = 0.0;
	private double targetDistance = 72.0; // Assume at least 6 feet away
	private boolean ledsON = false;

	public TargetSkateBot() {
		super();
		requires(OI.driveTrain);
		requires(OI.limelight);
		RioLogger.errorLog("TargetSkatebot Command Initialized");
	}

	@Override
	protected void execute() {
		// Turn on the LED's if they haven't been turned on before
		if (!ledsON) {
			OI.limelight.turnOnLED();
			ledsON = true;
		}

		// Driving
		Update_Limelight_Tracking();
		RioLogger.errorLog("Target identified is " + hasValidTarget);
		double leftPwr = driveCommand - steerCommand;
		double rightPwr = driveCommand + steerCommand;

		OI.driveTrain.setLeftPower(leftPwr);
		OI.driveTrain.setRightPower(rightPwr*-1.0);
		SmartDashboard.putNumber("LimeLightSteer", steerCommand);
		SmartDashboard.putNumber("LimelightDrive", driveCommand);
		SmartDashboard.putNumber("Right Power", rightPwr);
		SmartDashboard.putNumber("Left Power", leftPwr);
		//OI.driveTrain.drive();
	}

	@Override
	protected boolean isFinished() {
		boolean stop = false;
		// Stop when a) No target or b) Too close to the target
		if (!hasValidTarget) {
			stop = true;
		} else  if (targetDistance < 5.0) {
			stop = true;
		}
		return stop;
	}

	@Override
	protected void end() {
		OI.driveTrain.stop();
		OI.limelight.turnOffLED();
		ledsON = false;
		RioLogger.errorLog("TargetSkateBot command finished.");
	}

	/**
	 * This function implements a simple method of generating driving and steering
	 * commands based on the tracking data from a limelight camera.
	 */
	public void Update_Limelight_Tracking() {
		// These numbers must be tuned for your Robot! Be careful!
		// For testing make STEER_K = 0, no turning 
		final double STEER_K = 0.0; // 0.03 how hard to turn toward the target
		final double DRIVE_K = 1.0; // 0.26 how hard to drive fwd toward the target
		final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall

		hasValidTarget = OI.limelight.hasTargets();
		double ty = OI.limelight.y();
		double tx = OI.limelight.x();
		double ta = OI.limelight.targetArea();

		if (!hasValidTarget) {
			driveCommand = 0.0;
			steerCommand = 0.0;
			return;
		}

		// Start with proportional steering
		double steer_cmd = tx * STEER_K;
		steerCommand = steer_cmd;

		// try to drive forward until the target area reaches our desired area
		//double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
		calculateDistance(ty);
		double drive_cmd = 0.0;
		if (targetDistance > 24.0) {
			drive_cmd = MAX_SPEED;
		} else {
			drive_cmd = targetDistance * (MAX_SPEED / DISTANCE_TO_SLOW_DOWN);
		}
		// don't let the robot drive too fast into the goal
		// if (drive_cmd > MAX_SPEED) {
		// 	drive_cmd = MAX_SPEED;
		// }
		driveCommand = drive_cmd;
	}

	private void calculateDistance(double ty){
		double heightDiff = TARGET_HEIGHT - CAMERA_HEIGHT;
		double angle = ty + CAMERA_ANGLE;
		targetDistance = heightDiff / Math.tan(angle);
		SmartDashboard.putNumber("DistanceToTarget", targetDistance);
	}
}
