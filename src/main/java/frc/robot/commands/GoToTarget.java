/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.robot.Robot;
import frc.robot.RobotSettings;

public class GoToTarget extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GoToTarget() {
    //addSequential(new EnableVision(true));
    addSequential(new ElevateToHeight(1));
    addSequential(new WaitForVision());
    //addSequential(new RotateToTarget());
    //addSequential(new WaitCommand(0.2));
    //addParallel(new DriveUntilTarget(0.5, 12), 8);
    //addSequential(new EnableVision(false));
    //addSequential(new WaitForChildren());
    //addSequential(new DriveUntilDistance(0.8, 10), 0.5);

    addParallel(new JoystickDriveToTargetV2());
    addSequential(new WaitUntilTargetIsClose());
    //addSequential(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_PICKUP));
    addSequential(new WaitForChildren());
  }

  @Override
  protected void end() {
    Robot.disableVision = true;
  }

  @Override
  protected void initialize() {
    Robot.disableVision = false;
  }
  @Override
  protected void interrupted() {
    end();
  }
}
