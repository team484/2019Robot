/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotIO;

/**
 * Subsystem for the hatch panel manipulator
 */
public class HatchSub extends Subsystem {

  @Override
  public void initDefaultCommand() {
  }

  public static void setTop(Value v) {
    if (RobotIO.topHatchSolenoid == null) {
      return;
    }
    RobotIO.topHatchSolenoid.set(v);
  }

  public static void setBottom(Value v) {
    if (RobotIO.bottomHatchSolenoid == null) {
      return;
    }
    RobotIO.bottomHatchSolenoid.set(v);
  }

}
