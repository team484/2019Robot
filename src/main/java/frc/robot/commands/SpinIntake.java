/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotSettings;
import frc.robot.subsystems.IntakeSub;

/**
 * Spins the cargo intake wheels
 */
public class SpinIntake extends Command {
  public SpinIntake() {
    requires(Robot.intakeSub);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    IntakeSub.setSpeed(RobotSettings.INTAKE_MOTOR_SPEED);
  }

  // This command never finishes. Interrupt it to end it.
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    IntakeSub.setSpeed(0);
  }

}
