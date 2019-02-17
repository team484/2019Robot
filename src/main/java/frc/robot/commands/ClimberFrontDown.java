/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.ClimberSub;

/**
 * Retracts the front climber pistons to lower the robot
 */
public class ClimberFrontDown extends InstantCommand {

  @Override
  protected void execute() {
    ClimberSub.setFront(Value.kReverse);
  }

}