/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSub;

/**
 * Prevents the intake motor from spinning
 */
public class IntakeDoNothing extends Command {
  public IntakeDoNothing() {
    requires(Robot.intakeSub);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    IntakeSub.setSpeed(0);
  }

  // This command is never finished
  @Override
  protected boolean isFinished() {
    return false;
  }

}
