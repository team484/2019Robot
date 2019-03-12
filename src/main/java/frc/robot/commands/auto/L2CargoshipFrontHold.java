/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.DriveUntilDistance;
import frc.robot.commands.DriveUntilTarget;
import frc.robot.commands.ElevateToHeight;
import frc.robot.commands.EnableVision;
import frc.robot.commands.RotateAngle;
import frc.robot.commands.SetRocketOrCargo;
import frc.robot.commands.ShootHatch;

public class L2CargoshipFrontHold extends CommandGroup {
  /**
   * Add your docs here.
   */
  public L2CargoshipFrontHold(boolean isLeftSide) {
    addParallel(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_PICKUP));
    addSequential(new EnableVision(true));
    addSequential(new SetRocketOrCargo(false));
    addSequential(new DriveUntilDistance(0.8, 88.84));
    addParallel(new ElevateToHeight(0));
    addSequential(new RotateAngle(isLeftSide ? 45 : -45));
    addSequential(new DriveUntilDistance(0.8, 10.37));
    addSequential(new RotateAngle(isLeftSide ? -45 : 45));
    addSequential(new WaitCommand(0.3));
    addSequential(new DriveUntilTarget(0.3, 15), 1);
  }

  @Override
  protected boolean isFinished() {
    if (Math.abs(RobotIO.driverStick.getMagnitude()) > 0.2) {
      return true;
    }
    return super.isFinished();
  }
}
