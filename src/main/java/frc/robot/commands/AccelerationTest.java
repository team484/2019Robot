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

public class AccelerationTest extends Command {
  double voltage;
  public AccelerationTest(double voltage) {
    this.voltage = voltage;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.driveSub);
  }
  private long lastTime;
  private double lastRate;
  private long startTime;
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
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
  DriveSub.setVoltage(voltage, voltage);
  System.out.println((System.currentTimeMillis() - startTime) +" Accel: " + 1000.0 * accel/((double)diff) + "in/s^2" + " speed: " + newRate);
}

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    DriveSub.set(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
