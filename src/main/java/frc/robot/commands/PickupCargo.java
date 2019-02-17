/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

/**
 * Picks up the cargo from the ground and loads it into the shooter
 */
public class PickupCargo extends CommandGroup {

  public PickupCargo() {
    addParallel(new LowerIntake());
    addParallel(new SpinIntake());
    addSequential(new WaitForCargoInIntake());
    addParallel(new RaiseIntake());
    addParallel(new ShootCargo(0.6, false));
    addSequential(new WaitForCargoInShooter());
    addParallel(new IntakeDoNothing());
    addParallel(new CargoDoNothing());
    addSequential(new WaitCommand(1));
    addSequential(new CargoForwardUntilClear());
  }

}
