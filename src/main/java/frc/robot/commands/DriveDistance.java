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
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.subsystems.DriveSub;

/**
 * Drives a set distance using a PID loop
 */
public class DriveDistance extends Command {
	private static double speed = 1.0;
	private double speedMultiplier = 1.0;
	private static PIDController pid = new PIDController(RobotSettings.DRIVE_DISTANCE_KP,
			RobotSettings.DRIVE_DISTANCE_KI, RobotSettings.DRIVE_DISTANCE_KD, new PIDSource() {
				@Override
				public void setPIDSourceType(PIDSourceType pidSource) {
				}

				@Override
				public double pidGet() {
					return DriveSub.getDistance();
				}

				@Override
				public PIDSourceType getPIDSourceType() {
					return PIDSourceType.kDisplacement;
				}
			}, new PIDOutput() {
				@Override
				public void pidWrite(double output) {
					DriveSub.pidOut1 = output * speed;
					DriveSub.doublePIDDrive();
				}
			}, RobotSettings.DRIVE_PID_UPDATE_RATE);

	private static PIDController pid2 = new PIDController(RobotSettings.MAINTAIN_ANGLE_KP,
			RobotSettings.MAINTAIN_ANGLE_KI, RobotSettings.MAINTAIN_ANGLE_KD, new PIDSource() {

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
					DriveSub.pidOut2 = -output;
					DriveSub.doublePIDDrive();
				}
			}, RobotSettings.ROTATE_PID_UPDATE_RATE);

	private double setpoint;

	public DriveDistance(double setpoint) {
		requires(Robot.driveSub);
		this.setpoint = setpoint;
		speedMultiplier = 1.0;
	}

	public DriveDistance(double setpoint, double speedMultiplier) {
		requires(Robot.driveSub);
		this.setpoint = setpoint;
		this.speedMultiplier = speedMultiplier;
		requires(Robot.driveSub);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		speed = speedMultiplier;
		if (RobotIO.leftEncoder != null) {
			RobotIO.leftEncoder.reset();
		}
		if (RobotIO.rightEncoder != null) {
			RobotIO.rightEncoder.reset();
		}
		if (pid == null)
			return;
		pid.setAbsoluteTolerance(RobotSettings.DRIVE_PID_TOLERANCE);
		pid.setSetpoint(setpoint);

		double[] ypr = new double[3];
		if (RobotIO.imu != null) {
			RobotIO.imu.getYawPitchRoll(ypr);
		}
		if (pid2 == null)
			return;
		pid2.setSetpoint(ypr[0]);
		pid2.setAbsoluteTolerance(RobotSettings.ROTATE_PID_TOLERANCE);
		pid2.enable();
		pid.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return pid.onTarget() && Math.abs(DriveSub.getRate()) < RobotSettings.DRIVE_PID_RATE_TOLERANCE;
	}

	// Called once after isFinished returns true
	protected void end() {
		pid.disable();
		pid2.disable();
		DriveSub.set(0, 0);
	}
}
