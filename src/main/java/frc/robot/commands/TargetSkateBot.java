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

public class TargetSkateBot extends Command {
	private boolean m_LimelightHasValidTarget = false;
	private double m_LimelightDriveCommand = 0.0;
	private double m_LimelightSteerCommand = 0.0;
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
		RioLogger.errorLog("Target identified is " + m_LimelightHasValidTarget);

		double leftPwr = m_LimelightSteerCommand - m_LimelightDriveCommand;
		double rightPwr = m_LimelightSteerCommand - m_LimelightDriveCommand;
		OI.driveTrain.setLeftPower(leftPwr);
		OI.driveTrain.setRightPower(rightPwr);
		OI.driveTrain.drive();
	}


	@Override
	protected boolean isFinished() {
		boolean stop = false;
		// Stop when a) No target or b) Too close to the target
		if (!m_LimelightHasValidTarget) {
			stop = true;
		} else  {
			// Code to check distance
		}
		return stop;
	}

	@Override
	protected void end() {
		OI.driveTrain.stop();
		OI.limelight.turnOffLED();
		RioLogger.errorLog("TargetSkateBot command finished.");
		
	}

	/**
	 * This function implements a simple method of generating driving and steering
	 * commands based on the tracking data from a limelight camera.
	 */
	public void Update_Limelight_Tracking() {
		// These numbers must be tuned for your Robot! Be careful!
		final double STEER_K = 0.03; // how hard to turn toward the target
		final double DRIVE_K = 0.26; // how hard to drive fwd toward the target
		final double DESIRED_TARGET_AREA = 13.0; // Area of the target when the robot reaches the wall
		final double MAX_DRIVE = 0.7; // Simple speed limit so we don't drive too fast

		boolean hasTgt = OI.limelight.hasTargets();
		double tx = OI.limelight.x();
		double ty = OI.limelight.y();
		double ta = OI.limelight.targetArea();

		if (!hasTgt) {
			m_LimelightHasValidTarget = false;
			m_LimelightDriveCommand = 0.0;
			m_LimelightSteerCommand = 0.0;
			return;
		}

		m_LimelightHasValidTarget = true;

		// Start with proportional steering
		double steer_cmd = tx * STEER_K;
		m_LimelightSteerCommand = steer_cmd;

		// try to drive forward until the target area reaches our desired area
		double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

		// don't let the robot drive too fast into the goal
		if (drive_cmd > MAX_DRIVE) {
			drive_cmd = MAX_DRIVE;
		}
		m_LimelightDriveCommand = drive_cmd;
	}
}
