/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotIO;
import frc.robot.RobotSettings;
import frc.robot.commands.JoystickDrive;

/**
 * Subsystem for the drivetrain
 */
public class DriveSub extends Subsystem {
  private static double lastLDistReset = 0;
  private static double lastRDistReset = 0;

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickDrive());
  }

  public static void set(double speed, double rot) {
    if (RobotIO.diffDrive == null) {
      return;
    }
    RobotIO.diffDrive.arcadeDrive(speed, rot);
  }

  public static void tankDrive(double left, double right) {
    if (RobotIO.diffDrive == null) {
      return;
    }
    RobotIO.diffDrive.tankDrive(left, right);
  }

  public static void setVoltageCompensation(boolean enabled) {
    if (enabled) {
      RobotIO.leftMotor1.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
      RobotIO.leftMotor2.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
      RobotIO.leftMotor3.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
      RobotIO.rightMotor1.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
      RobotIO.rightMotor2.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
      RobotIO.rightMotor3.enableVoltageCompensation(RobotSettings.VOLTAGE_TARGET);
    } else {
      RobotIO.leftMotor1.disableVoltageCompensation();
      RobotIO.leftMotor2.disableVoltageCompensation();
      RobotIO.leftMotor3.disableVoltageCompensation();
      RobotIO.rightMotor1.disableVoltageCompensation();
      RobotIO.rightMotor2.disableVoltageCompensation();
      RobotIO.rightMotor3.disableVoltageCompensation();
    }
  }

  public static double getDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  public static double getLeftDistance() {
    if (Math.abs(RobotIO.leftEncoder.getDistance()) > 0.7 * Math.abs(getLeftIntegratedEncDistance())) {
      return RobotIO.leftEncoder.getDistance() - lastLDistReset;
    }
    System.err.println("Possible error with left encoder");
    return getLeftIntegratedEncDistance() - lastLDistReset;
  }

  public static double getRightDistance() {
    if (Math.abs(RobotIO.rightEncoder.getDistance()) > 0.7 * Math.abs(getRightIntegratedEncDistance())) {
      return RobotIO.rightEncoder.getDistance() - lastRDistReset;
    }
    System.err.println("Possible error with right encoder");
    return getRightIntegratedEncDistance() - lastRDistReset;
  }

  public static void resetDistance() {
    lastLDistReset += getLeftDistance();
    lastRDistReset += getRightDistance();
  }

  private static double lastLd1 = 0;

  private static double getLeftIntegratedEncDistance() {
    double d1 = RobotIO.leftMotor1.getEncoder().getPosition() * RobotSettings.DRIVE_MOTOR_ENC_DPP;
    if (d1 == 0) {
      return lastLd1;
    }
    lastLd1 = d1;
    return d1;
  }

  private static double lastRd1 = 0;

  private static double getRightIntegratedEncDistance() {
    double d1 = -RobotIO.rightMotor1.getEncoder().getPosition() * RobotSettings.DRIVE_MOTOR_ENC_DPP;
    if (d1 == 0) {
      return lastRd1;
    }
    lastRd1 = d1;
    return d1;
  }

  /**
   * Gets the current heading (yaw) of the gyro.
   * 
   * @return - yaw in degrees
   */
  public static double getHeading() {
    double[] ypr = new double[3];
    RobotIO.imu.getYawPitchRoll(ypr);
    return ypr[0];
  }
  
  public static double pidOut1, pidOut2;

  public static void doublePIDDrive() {
    set(pidOut1, pidOut2);
  }

  public static double getRate() {
    double leftEncRate = RobotIO.leftEncoder.getRate();
    double rightEncRate = RobotIO.rightEncoder.getRate();
    return (leftEncRate > rightEncRate) ? leftEncRate : rightEncRate;
  }
}
