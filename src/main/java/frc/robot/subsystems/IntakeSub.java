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
import frc.robot.commands.IntakeDoNothing;

/**
 * Subsystem for the cargo intake
 */
public class IntakeSub extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new IntakeDoNothing());
  }

  public static void setSpeed(double speed) {
    if (RobotIO.intakeMotor == null)
      return;
    RobotIO.intakeMotor.set(speed);
  }

  public static void setArm(Value v) {
    if (RobotIO.intakeSolenoid == null) {
      return;
    }
    RobotIO.intakeSolenoid.set(v);
  }

}
