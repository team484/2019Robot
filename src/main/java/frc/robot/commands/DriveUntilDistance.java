/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LEDSub;

/**
 * Drives at a set speed until a set distance is passed
 */
public class DriveUntilDistance extends Command {
  private double speed;
  private double distance;
  private boolean onlyForCargo;

  public DriveUntilDistance(double speed, double distance) {
    this.speed = speed;
    this.distance = distance;
    requires(Robot.driveSub);
  }

  public DriveUntilDistance(double speed, double distance, boolean onlyForCargo) {
    this.speed = speed;
    this.distance = distance;
    requires(Robot.driveSub);
    this.onlyForCargo = onlyForCargo;

  }
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    DriveSub.resetDistance();
    DriveSub.setVoltageCompensation(true);
    LEDSub.actionsInProgress++;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    DriveSub.set(speed, 0);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (onlyForCargo && SetRocketOrCargo.atRocket) {
      return true;
    }
    if (Math.abs(RobotIO.driverStick.getMagnitude()) > 0.2) {
      return true;
    }
    return (DriveSub.getDistance() > distance && speed > 0) || (DriveSub.getDistance() < distance && speed < 0);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    DriveSub.setVoltageCompensation(false);
    DriveSub.set(0, 0);
    LEDSub.actionsInProgress--;
  }
  @Override
  protected void interrupted() {
    end();
  }

}
