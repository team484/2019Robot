/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.LEDSub;

public class RaiseElevator extends Command {
  private double startHeight;
  public RaiseElevator() {
    requires(Robot.elevatorSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startHeight = ElevatorSub.getHeight();
    LEDSub.actionsInProgress++;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    ElevatorSub.set(0.45);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ElevatorSub.getHeight() > startHeight + 2;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    ElevatorSub.set(0);
    LEDSub.actionsInProgress--;
  }
  @Override
  protected void interrupted() {
    end();
  }
}
