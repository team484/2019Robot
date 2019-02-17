/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.HatchSub;

/**
 * Extends the hatch grabber
 */
public class ExtendHatch extends InstantCommand {
  private boolean checkForRocket = false;
  public ExtendHatch() {
    checkForRocket = false;
  }

  public ExtendHatch(boolean onlyIfAtRocket) {
    checkForRocket = onlyIfAtRocket;
  }

  @Override
  protected void execute() {
    if (!checkForRocket || SetRocketOrCargo.atRocket) {
      HatchSub.setTop(Value.kForward);
    }
  }

}
