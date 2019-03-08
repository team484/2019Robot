/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSub;

/**
 * Extends the front climber pistons to raise the robot
 */
public class ClimberFrontToggle extends InstantCommand {

  @Override
  protected void execute() {
    if (System.currentTimeMillis() - Robot.matchStartTime < 120*1000 && DriverStation.getInstance().isFMSAttached()) {
      return; //Only allow the robot to climb in the last 30 seconds
    }
    ClimberSub.toggleFront();
  }

}
