/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.JoystickElevator;

/**
 * Subsystem for the elevator
 */
public class ElevatorSub extends Subsystem {

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickElevator());
  }

  public static void set(double speed) {
    set(speed, false);
  }

  public static void set(double speed, boolean managed) {
    if (RobotIO.elevatorMotorLeft == null || RobotIO.elevatorMotorRight == null) {
      return;
    }
    double height = getHeight();
    double rate = getRate();
    if ((getHeight() < 2 && speed < 0.08) || (height >= RobotSettings.ELEVATOR_UP_THRESHOLD && speed > -0.08)) {
      RobotIO.elevatorMotorLeft.set(0);
      RobotIO.elevatorMotorRight.set(0);
      return;
    }
    double minRate = -height * RobotSettings.ELEVATOR_MAX_DECEL_RATE;
    SmartDashboard.putNumber("MIN RATE", minRate);
    double maxRate = (RobotSettings.ELEVATOR_UP_THRESHOLD - height) * RobotSettings.ELEVATOR_MAX_DECEL_RATE;
    SmartDashboard.putNumber("MAX RATE", maxRate);
    SmartDashboard.putNumber("RATE", rate);

    if (minRate < 0 && rate < minRate && speed < 0 && managed) {
      RobotIO.elevatorMotorLeft.set(RobotSettings.ELEVATOR_GRAVITY_COMP);
      RobotIO.elevatorMotorRight.set(-RobotSettings.ELEVATOR_GRAVITY_COMP);
    } else if (maxRate > 0 && rate > maxRate && speed > 0 && managed) {
      RobotIO.elevatorMotorLeft.set(RobotSettings.ELEVATOR_GRAVITY_COMP);
      RobotIO.elevatorMotorRight.set(-RobotSettings.ELEVATOR_GRAVITY_COMP);
    } else {
      RobotIO.elevatorMotorLeft
          .set(speed * RobotSettings.ELEVATOR_SPEED_MULTIPLYER + RobotSettings.ELEVATOR_GRAVITY_COMP);
      RobotIO.elevatorMotorRight
          .set(-speed * RobotSettings.ELEVATOR_SPEED_MULTIPLYER - RobotSettings.ELEVATOR_GRAVITY_COMP);
    }
  }

  private static double lastHeight = 0;

  public static double getHeight() {
    double left = (RobotIO.elevatorMotorLeft == null) ? 0
        : RobotIO.elevatorMotorLeft.getEncoder().getPosition() * RobotSettings.ELEVATOR_ENCODER_DPP;
    double right = (RobotIO.elevatorMotorRight == null) ? 0
        : -RobotIO.elevatorMotorRight.getEncoder().getPosition() * RobotSettings.ELEVATOR_ENCODER_DPP;
    if (left < 0) {
      RobotIO.elevatorMotorLeft.setEncPosition(0);
    }
    if (right < 0) {
      RobotIO.elevatorMotorRight.setEncPosition(0);
    }
    if (RobotIO.elevatorDownSwitch.get() == RobotSettings.ELEVATOR_SWITCH_DOWN_STATE) {
      RobotIO.elevatorMotorLeft.setEncPosition(0);
      RobotIO.elevatorMotorRight.setEncPosition(0);
      return 0;
    }
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
    lastHeight = (left + right) / 2.0;
    return lastHeight;
  }

  public static boolean isUp() {
    return getHeight() > RobotSettings.ELEVATOR_UP_THRESHOLD;
  }

  public static double getRate() {
    double leftRate = (RobotIO.elevatorMotorLeft == null) ? 0
        : RobotIO.elevatorMotorLeft.getEncoder().getVelocity() * RobotSettings.ELEVATOR_ENCODER_DPP / 60.0;
    double rightRate = (RobotIO.elevatorMotorRight == null) ? 0
        : -RobotIO.elevatorMotorRight.getEncoder().getVelocity() * RobotSettings.ELEVATOR_ENCODER_DPP / 60.0;
    return (leftRate > rightRate) ? leftRate : rightRate;
  }
}
