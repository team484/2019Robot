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

/**
 * This command runs until cargo is inside the intake
 */
public class WaitForCargoInIntake extends Command {

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (RobotIO.intakeSensor.getAverageVoltage() > RobotSettings.INTAKE_SENSOR_VOLTAGE);
  }

}
