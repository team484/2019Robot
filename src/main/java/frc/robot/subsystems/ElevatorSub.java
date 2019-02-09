/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.JoystickElevator;

/**
 * Add your docs here.
 */
public class ElevatorSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    
    setDefaultCommand(new JoystickElevator());
  }

  public static void set(double speed) {
    RobotIO.elevatorMotorLeft.getPIDController().setReference((speed*RobotSettings.ELEVATOR_SPEED_MULTIPLYER + RobotSettings.ELEVATOR_GRAVITY_COMP), ControlType.kVoltage);
    RobotIO.elevatorMotorRight.getPIDController().setReference((-speed*RobotSettings.ELEVATOR_SPEED_MULTIPLYER - RobotSettings.ELEVATOR_GRAVITY_COMP), ControlType.kVoltage);
  }

  private static double lastHeight = 0;
  public static double getHeight() {
    double left = RobotIO.elevatorMotorLeft.getEncoder().getPosition() * RobotSettings.ELEVATOR_ENCODER_DPP;
    double right = -RobotIO.elevatorMotorRight.getEncoder().getPosition() * RobotSettings.ELEVATOR_ENCODER_DPP;
    if (left == 0 && right == 0) {
      return lastHeight;
    }
    if (left == 0) {
      lastHeight = right;
      return right;
    }
    if (right == 0) {
      lastHeight = left;
      return left;
    }
    lastHeight = (left+right)/2.0;
    return lastHeight;
  }

  public static boolean isUp() {
    return getHeight() > RobotSettings.ELEVATOR_UP_THRESHOLD;
  }

  public static double getRate() {
    double leftRate = RobotIO.elevatorMotorLeft.getEncoder().getVelocity() * RobotSettings.ELEVATOR_ENCODER_DPP;
    double rightRate = -RobotIO.elevatorMotorRight.getEncoder().getVelocity() * RobotSettings.ELEVATOR_ENCODER_DPP;
    return (leftRate > rightRate) ? leftRate : rightRate;
  }
}
