/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSub;

/**
 * Runs the robot at a given speed and prints the acceleration to the console
 */
public class AccelerationTest extends Command {
  double speed;

  public AccelerationTest(double speed) {
    this.speed = speed;
    requires(Robot.driveSub);
  }

  private long lastTime;
  private double lastRate;
  private long startTime;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    DriveSub.setVoltageCompensation(true);
    lastTime = System.currentTimeMillis();
    startTime = lastTime;
    lastRate = DriveSub.getRate();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    long newTime = System.currentTimeMillis();
    long diff = newTime - lastTime;
    lastTime = newTime;
    double newRate = DriveSub.getRate();
    double accel = newRate - lastRate;
    lastRate = newRate;
    DriveSub.set(speed, 0);
    System.out.println((System.currentTimeMillis() - startTime) + " Accel: " + 1000.0 * accel / ((double) diff)
        + "in/s^2" + " speed: " + newRate);
  }

  // This command is never finished. To stop, call interrupt.
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    DriveSub.setVoltageCompensation(false);
    DriveSub.set(0, 0);
  }

}
