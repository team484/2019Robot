/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotIO;
import frc.robot.commands.ClimberDoNothing;

/**
 * Add your docs here.
 */
public class ClimberSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ClimberDoNothing());
  }

  public static void setSpeed(double speed) {
    RobotIO.leftClimberMotor.set(speed);
    RobotIO.rightClimberMotor.set(speed);

  }
}
