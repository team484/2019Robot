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
import frc.robot.subsystems.ElevatorSub;

/**
 * Drives at a set speed until the vision target is reached
 */
public class JoystickDriveToTargetV2 extends Command {
  private int i = 0;
  private static double lastHeading = 0;

private static PIDController pid = new PIDController(RobotSettings.MAINTAIN_ANGLE_KP,
  RobotSettings.MAINTAIN_ANGLE_KI, RobotSettings.MAINTAIN_ANGLE_KD, new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public double pidGet() {
      if (Robot.targetFound && ElevatorSub.getHeight() < 4 && Robot.targetDistance > 20 && Robot.targetGyroAngle != 0) { 
        lastHeading = Robot.targetGyroAngle;
      }
      return DriveSub.getHeading() - lastHeading;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  }, new PIDOutput() {

    @Override
    public void pidWrite(double output) {
      double joystick = RobotIO.driverStick.getY();
      if (joystick > 0.65) joystick = 0.65;
      if (joystick < -0.65) joystick = -0.65;
      DriveSub.set(-joystick, output);
    }
  }, RobotSettings.ROTATE_PID_UPDATE_RATE);
  public JoystickDriveToTargetV2() {
    requires(Robot.driveSub);
    pid.setOutputRange(-0.65, 0.65);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pid.setSetpoint(0);
    lastHeading = DriveSub.getHeading();
    if (Robot.targetFound && ElevatorSub.getHeight() < 4) { 
      lastHeading = Robot.targetGyroAngle;
    }
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
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    pid.disable();
    DriveSub.setVoltageCompensation(false);
    DriveSub.set(0, 0);
  }
  @Override
  protected void interrupted() {
    end();
  }
}
