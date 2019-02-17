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
 * Subsystem for the climber pistons
 */
public class ClimberSub extends Subsystem {

  @Override
  public void initDefaultCommand() {
  }

  public static void set(Value direction) {
    if (RobotIO.climberBackSolenoid == null) {
      return;
    }
    RobotIO.climberBackSolenoid.set(direction);
  }

  public static void setFront(Value direction) {
    if (RobotIO.climberFrontSolenoid == null) {
      return;
    }
    RobotIO.climberFrontSolenoid.set(direction);
  }
}
