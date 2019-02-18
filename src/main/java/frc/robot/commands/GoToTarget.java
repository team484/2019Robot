/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class GoToTarget extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GoToTarget() {
    addSequential(new ElevateToHeight(0));
    addSequential(new RotateToTarget());
    addSequential(new WaitCommand(0.2));
    addSequential(new DriveUntilTarget(0.5, 12), 8);
    addSequential(new DriveUntilDistance(0.8, 10), 0.5);
  }
}
