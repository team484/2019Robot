/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ShootHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootHatch() {
    addSequential(new ReleaseHatch(true));
    addSequential(new ExtendHatch(true));
    addSequential(new WaitCommand(0.5));
    addSequential(new DriveUntilDistanceCollisionStop(0.3, 10)); //make sure we are against ship
    addSequential(new ReleaseHatch());
    addSequential(new WaitCommand(0.5));
    addSequential(new DriveUntilDistance(-0.3, -0.2));
    addSequential(new LowerElevator());
    addSequential(new WaitCommand(0.5));
    addSequential(new DriveUntilDistance(-0.3, -3));
  }
}
