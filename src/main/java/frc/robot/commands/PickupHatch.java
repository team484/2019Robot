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
import frc.robot.RobotSettings;
import frc.robot.subsystems.LEDSub;

public class PickupHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PickupHatch() {
    addSequential(new ReleaseHatch());
    addSequential(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_PICKUP));
    addSequential(new DriveUntilDistanceCollisionStop(0.3, 10), 1.0); //make sure we are against ship
    addSequential(new ExtendHatch());
    addSequential(new WaitCommand(0.1));
    addSequential(new GrabHatch());
    addSequential(new WaitCommand(0.1));
    addSequential(new RetractHatch());

  }

  @Override
  public void initialize() {
    LEDSub.actionsInProgress++;
  }

  @Override
  public void end() {
    LEDSub.actionsInProgress--;
  }
  @Override
  protected void interrupted() {
    end();
  }
}
