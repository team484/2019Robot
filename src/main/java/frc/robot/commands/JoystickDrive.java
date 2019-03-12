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
import frc.robot.subsystems.ElevatorSub;

/**
 * Uses driver joystick inputs to control the drivetrain
 */
public class JoystickDrive extends Command {
  public JoystickDrive() {
    requires(Robot.driveSub);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double y = RobotIO.driverStick.getY();
    if (ElevatorSub.getHeight() > 35) {
      y /= 2.0;
    }
    double x = RobotIO.driverStick.getX() * 0.8;
    if (x > 0.8) x = 0.8;
    if (x < -0.8) x = -0.8;
    DriveSub.set(-y * 0.7, x);
  }

  // This command is never finished
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after command is interrupted
  @Override
  protected void end() {
    DriveSub.set(0,0);
  }
  @Override
  protected void interrupted() {
    end();
  }

}
