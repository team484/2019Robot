/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.subsystems.DriveSub;

/**
 * Rotates the robot a set angle (in degrees)
 */
public class RotateToTarget extends Command {
	private static PIDController pid = new PIDController(RobotSettings.ROTATE_ANGLE_KP, RobotSettings.ROTATE_ANGLE_KI,
			RobotSettings.ROTATE_ANGLE_KD, new PIDSource() {

				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {
				}

				@Override
				public double pidGet() {
					if (RobotIO.imu == null)
						return 0;
					double[] ypr = new double[3];
					RobotIO.imu.getYawPitchRoll(ypr);
					return ypr[0];
				}

				@Override
				public PIDSourceType getPIDSourceType() {
					return PIDSourceType.kDisplacement;
				}
			}, new PIDOutput() {

				@Override
				public void pidWrite(double output) {
					DriveSub.set(0, output);
				}
			}, RobotSettings.ROTATE_PID_UPDATE_RATE);
	private double setpoint;

	public RotateToTarget() {
		requires(Robot.driveSub);
		pid.setOutputRange(-0.5, 0.5);
		requires(Robot.driveSub);
	}
	private boolean targetFound = true;
	protected void initialize() {
		setpoint = Robot.targetAngle;
		targetFound = Robot.targetFound;
		DriveSub.setVoltageCompensation(true);
		double[] ypr = new double[3];
		if (RobotIO.imu != null) {
			RobotIO.imu.getYawPitchRoll(ypr);
		}
		if (pid == null)
			return;
		pid.setSetpoint(setpoint + ypr[0]);
		pid.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
		pid.enable();
	}

	double lastYaw = 0;
	protected boolean isFinished() {
		double newHeading = DriveSub.getHeading();
		if (!targetFound) {
			return true;
		}
		if (pid == null)
			return true;
		if (Math.abs(pid.getError()) < 1 && Math.abs(newHeading - lastYaw) < 0.1) {
			return true;
		}
		lastYaw = newHeading;
		return false;
	}

	protected void end() {
		if (pid != null) {
			pid.disable();
		}
		DriveSub.setVoltageCompensation(false);
		DriveSub.set(0, 0);
	}
}
