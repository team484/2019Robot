/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Set if the robot is currently at a rocket ship or cargo ship
 */
public class SetRocketOrCargo extends InstantCommand {
  public static boolean atRocket = true;
  private boolean valueToSet;
  public SetRocketOrCargo(boolean atRocket) {
    valueToSet = atRocket;
  }

  @Override
  public void execute() {
    SetRocketOrCargo.atRocket = valueToSet;
  }
}
