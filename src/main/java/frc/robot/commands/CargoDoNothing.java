/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSub;

/**
 * Keeps cargo shooter motor off
 */
public class CargoDoNothing extends Command {
  public CargoDoNothing() {
    requires(Robot.cargoSub);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    CargoSub.set(0);
  }

  // This command is never finished
  @Override
  protected boolean isFinished() {
    return false;
  }

}
