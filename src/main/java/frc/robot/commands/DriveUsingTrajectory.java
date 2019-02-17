/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSettings;
import frc.robot.subsystems.DriveSub;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * This class/command is designed to follow a given trajectory. The trajectory
 * being used is assigned in the class constructor. The trajectory must have
 * been generated by the GenerateTrajectory class before the instance of this
 * class is created. It is suggested to run the GenerateTrajectory class at
 * least 30 seconds before this is run to ensure the trajectory is generated and
 * saved. Trajectories are persistent across reboots.
 */
public class DriveUsingTrajectory extends Command {
	private static final String SAVE_DIR = RobotSettings.SAVE_DIR_TRAJECTORIES;

	private static final double KP = RobotSettings.KP;
	private static final double KI = RobotSettings.KI;
	private static final double KD = RobotSettings.KD;
	private static final double VELOCITY_RATIO = RobotSettings.VELOCITY_RATIO;
	private static final double ACCELERATION_GAIN = RobotSettings.ACCELERATION_GAIN;

	private static final int LEFT_ENC_TIC_PER_ROT = RobotSettings.LEFT_ENC_TIC_PER_ROT;
	private static final int RIGHT_ENC_TIC_PER_ROT = RobotSettings.RIGHT_ENC_TIC_PER_ROT;
	private static final double WHEEL_DIAMETER = RobotSettings.WHEEL_DIAMETER;

	private EncoderFollower left;
	private EncoderFollower right;
	private final String name;

	/**
	 * Create a new DriveUsingTrajectory command. This command will follow a
	 * trajectory of a given name.
	 * 
	 * @param name - Name of trajectory to follow.
	 */
	public DriveUsingTrajectory(String name) {
		requires(Robot.driveSub);
		this.name = name;
	}

	double startHeading;

	/*
	 * Resets the encoders and loads the trajectories from disk.
	 */
	protected void initialize() {
		DriveSub.setVoltageCompensation(true);
		File leftTrajectoryFile = new File(SAVE_DIR + name + "/LeftTrajectory.traj");
		Trajectory leftTrajectory = Pathfinder.readFromFile(leftTrajectoryFile);
		File rightTrajectoryFile = new File(SAVE_DIR + name + "/RightTrajectory.traj");
		Trajectory rightTrajectory = Pathfinder.readFromFile(rightTrajectoryFile);
		left = new EncoderFollower(leftTrajectory);
		right = new EncoderFollower(rightTrajectory);
		left.configureEncoder(0, LEFT_ENC_TIC_PER_ROT, WHEEL_DIAMETER);
		left.configurePIDVA(KP, KI, KD, VELOCITY_RATIO, ACCELERATION_GAIN);
		right.configureEncoder(0, RIGHT_ENC_TIC_PER_ROT, WHEEL_DIAMETER);
		right.configurePIDVA(KP, KI, KD, VELOCITY_RATIO, ACCELERATION_GAIN);

		DriveSub.resetDistance();
		startHeading = DriveSub.getHeading();
	}

	/*
	 * Called repeatedly when this Command is scheduled to run Sets the drivetrain
	 * output based on EncoderFollowers and heading error.
	 */
	protected void execute() {
		double outputL = left.calculate((int) (1000 * DriveSub.getLeftDistance()));
		double outputR = right.calculate((int) (1000 * DriveSub.getRightDistance()));
		double actualHeading = DriveSub.getHeading() - startHeading + 90.0;
		double desiredHeading = Pathfinder.r2d(left.getHeading());

		double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - actualHeading);
		double turn = 4.0 * (-1.0 / 80.0) * angleDifference;

		DriveSub.tankDrive((outputL + turn), (outputR - turn));
	}

	// Command is finished when both trajectories have been completed
	protected boolean isFinished() {
		return left.isFinished() && right.isFinished();
	}

	// For safety, sets motor output to 0.
	protected void end() {
		DriveSub.setVoltageCompensation(false);
		DriveSub.set(0, 0);
		left = null;
		right = null;
	}
}
