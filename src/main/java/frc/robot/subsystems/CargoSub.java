/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotIO;
import frc.robot.commands.CargoForwardUntilClear;

/**
 * Subsystem for cargo shooter
 */
public class CargoSub extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new CargoForwardUntilClear());
  }

  public static void set(double speed) {
    if (RobotIO.cargoMotor == null) {
      return;
    }
    RobotIO.cargoMotor.set(speed);
  }
}
