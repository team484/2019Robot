/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.subsystems.LEDSub;

public class ShootHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public ShootHatch() {
    addSequential(new GrabHatch());
    addParallel(new DriveUntilDistance(0.4, 10),0.5);
    addSequential(new ExtendHatch(true));
    addSequential(new WaitCommand(0.4));
    addSequential(new ReleaseHatch());
    addSequential(new WaitCommand(0.5));
    addSequential(new RetractHatch());
    addSequential(new WaitCommand(0.5));
    addSequential(new GrabHatch());
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
