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
 * Drives at a set speed until the vision target is reached
 */
public class DriveUntilTarget extends Command {
  private double speed;
  private double distance;
  private int i = 0;

private static PIDController pid = new PIDController(RobotSettings.MAINTAIN_ANGLE_KP,
  RobotSettings.MAINTAIN_ANGLE_KI, RobotSettings.MAINTAIN_ANGLE_KD, new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public double pidGet() {
      return DriveSub.getHeading();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  }, new PIDOutput() {

    @Override
    public void pidWrite(double output) {
      DriveSub.pidOut2 = output;
      DriveSub.doublePIDDrive();
      System.out.println("RUNNING " + output + ", " + DriveSub.pidOut1);
    }
  }, RobotSettings.ROTATE_PID_UPDATE_RATE);
  public DriveUntilTarget(double speed, double distance) {
    this.speed = speed;
    this.distance = distance;
    requires(Robot.driveSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //SmartDashboard.putData(pid);
    DriveSub.pidOut1 = this.speed;
    DriveSub.pidOut2 = 0;
    pid.setSetpoint(DriveSub.getHeading() + Robot.targetAngle);
    pid.reset();
    pid.enable();
    DriveSub.setVoltageCompensation(true);
    i = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    i++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if (Math.abs(RobotIO.driverStick.getMagnitude()) > 0.2) {
      return true;
    }
    if (Robot.targetDistance < distance && Robot.targetFound) {
      //return true;
    }
    return speed <= 2 && i >= 50;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pid.disable();
    DriveSub.setVoltageCompensation(false);
    DriveSub.set(0, 0);
  }

}
