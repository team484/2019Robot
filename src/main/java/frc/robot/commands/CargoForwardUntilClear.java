/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotSettings;

/**
 * Slowly moves the cargo forward in the shooter until it no longer interferes
 * with the elevator
 */
public class CargoForwardUntilClear extends CommandGroup {

  public CargoForwardUntilClear() {
    addParallel(new ShootCargo(RobotSettings.CARGO_INTAKE_SPEED, true));
    addSequential(new WaitForCargoOutOfShooter());
    addParallel(new CargoDoNothing());
    addSequential(new WaitCommand(1));
  }
}
