/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.subsystems.LEDSub;

/**
 * This command runs until cargo is no longer sticking out of the back of the
 * shooter.
 */
public class WaitForCargoOutOfShooter extends Command {
  @Override
  protected void initialize() {
    LEDSub.actionsInProgress++;
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (RobotIO.cargoSensor.getAverageVoltage() < RobotSettings.CARGO_SENSOR_OUT_VOLTAGE);
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
