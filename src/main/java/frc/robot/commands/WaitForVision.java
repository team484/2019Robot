/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.LEDSub;

/**
 * Waits for vision pipeline to execute before continuing
 */
public class WaitForVision extends Command {
  @Override
  protected void initialize() {
    LEDSub.actionsInProgress++;
  }
  @Override
  protected boolean isFinished() {
    return (!Robot.disableVision) && Robot.visionUpToDate;
  }
  @Override
  protected void end() {
    LEDSub.actionsInProgress--;
  }
  @Override
  protected void interrupted() {
    end();
  }
}
