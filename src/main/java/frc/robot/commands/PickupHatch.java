/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotSettings;

public class PickupHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickupHatch() {
    addSequential(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_PICKUP));
    addSequential(new ReleaseHatch());
    addSequential(new DriveUntilDistanceCollisionStop(0.3, 10)); //make sure we are against ship
    addSequential(new DriveUntilDistance(-0.3, -0.2));
    addSequential(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_PICKUP_RAISED));
    addSequential(new GrabHatch());
    addSequential(new DriveUntilDistance(-0.3, -3));
  }
}
