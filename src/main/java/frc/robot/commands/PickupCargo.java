/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PickupCargo extends CommandGroup {

  public PickupCargo() {

    addParallel(new LowerIntake());
    addParallel(new ShootCargo());
    addParallel(new SpinIntake());
    addSequential(new WaitForCargoInIntake());
    addParallel(new RaiseIntake());
    addSequential(new WaitForCargoInShooter());
    addParallel(new CargoDoNothing());
    addSequential(new IntakeDoNothing());

  }
  
}
