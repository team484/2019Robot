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

  public static void setBack(Value direction) {
    if (RobotIO.climberBackSolenoid == null) {
      return;
    }
    RobotIO.climberBackSolenoid.set(direction);
  }

  public static void toggleBack() {
    if (RobotIO.climberBackSolenoid == null) {
      return;
    }
    if (RobotIO.climberBackSolenoid.get() == Value.kReverse) {
      RobotIO.climberBackSolenoid.set(Value.kForward);
    } else {
      RobotIO.climberBackSolenoid.set(Value.kReverse);
    }
  }

  public static void setFront(Value direction) {
    if (RobotIO.climberFrontSolenoid == null) {
      return;
    }
    RobotIO.climberFrontSolenoid.set(direction);
  }

  public static void toggleFront() {
    if (RobotIO.climberFrontSolenoid == null) {
      return;
    }
    if (RobotIO.climberFrontSolenoid.get() == Value.kReverse) {
      RobotIO.climberFrontSolenoid.set(Value.kForward);
    } else {
      RobotIO.climberFrontSolenoid.set(Value.kReverse);
    }
  }
}
