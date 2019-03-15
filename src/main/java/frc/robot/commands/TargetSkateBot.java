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
	private static double DISTANCE_TO_SLOW_DOWN = 24.0;
	private static double MAX_SPEED = 0.7; // Drive Motor Limit. Used to cap the speed of the bot
	private static double DESIRED_TARGET_AREA = 17; // Area of the target when the robot reaches the wall
	private static double DRIVE_K = 0.75; // 0.26 how hard to drive fwd toward the target
	private static double STEER_K = 0.01; // 0.03 how hard to turn toward the target

	// The following fields are updated by the LimeLight Camera
	private boolean hasValidTarget = false;
	private double driveCommand = 0.0;
	private double steerCommand = 0.0;
	private double speedToTarget = 0.0;

	// The following fields are updated by the state of the Command
	private boolean ledsON = false;
	private boolean isTargeting = false;

	public TargetSkateBot() {
		super();
		requires(OI.driveTrain);
		requires(OI.limelight);
		initializeCommand();
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

		double leftPwr = (driveCommand - steerCommand) * -1.0;
		double rightPwr = (driveCommand + steerCommand) * -1.0;

		OI.driveTrain.setLeftPower(leftPwr);
		OI.driveTrain.setRightPower(rightPwr);
		OI.driveTrain.drive();
		SmartDashboard.putNumber("LimeLight.RightPower", rightPwr);
		SmartDashboard.putNumber("LimeLight.LeftPower", leftPwr);
	}

	@Override
	protected boolean isFinished() {
		boolean stop = false;
		if (isTargeting) {
			if (!hasValidTarget) {
				stop = true;
			}
			if (speedToTarget < 0.01) {
				stop = true;
			}
		}
		return stop;
	}

	@Override
	protected void end() {
		OI.driveTrain.stop();
		OI.limelight.turnOffLED();
		RioLogger.errorLog("TargetSkateBot command finished.");
		initializeCommand();
	}

	/**
	 * This function implements a simple method of generating driving and steering
	 * commands based on the tracking data from a limelight camera.
	 */
	public void Update_Limelight_Tracking() {
		driveCommand = 0.0;
		steerCommand = 0.0;

		hasValidTarget = OI.limelight.hasTargets();
		if (!hasValidTarget) {
			return;
		}
		isTargeting = true;
		// double ty = OI.limelight.y();
		double tx = OI.limelight.x();
		double ta = OI.limelight.targetArea();

		// Start with proportional steering
		steerCommand = tx * STEER_K;
		SmartDashboard.putNumber("Limelight.SteerCommand", steerCommand);

		// try to drive forward until the target area reaches our desired area
		speedToTarget = (DESIRED_TARGET_AREA - ta) * DRIVE_K;
		SmartDashboard.putNumber("Limelight.SpeedToTarget", speedToTarget);

		// Adjust Drive Speed, based on how close the bot is to the target
		if (speedToTarget > DISTANCE_TO_SLOW_DOWN) {
			driveCommand = MAX_SPEED;
		} else {
			driveCommand = speedToTarget * (MAX_SPEED / DISTANCE_TO_SLOW_DOWN);
		}
		SmartDashboard.putNumber("Limelight.DriveCommand", driveCommand);
	}

	private void initializeCommand() {
		ledsON = false;
		isTargeting = false;
	}
}
