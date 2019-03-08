/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotSettings;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.LEDSub;

/**
 * Raises/lowers the elevator to the specified height
 */
public class ElevateToHeight extends Command {
  public static boolean isElevating = false;
  private static PIDController pid = new PIDController(RobotSettings.ELEVATOR_UP_KP, RobotSettings.ELEVATOR_UP_KI,
      RobotSettings.ELEVATOR_UP_KD, new PIDSource() {

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
        }

        @Override
        public double pidGet() {
          System.out.println(ElevatorSub.getHeight());
          return ElevatorSub.getHeight();
        }

        @Override
        public PIDSourceType getPIDSourceType() {
          return PIDSourceType.kDisplacement;
        }
      }, new PIDOutput() {

        @Override
        public void pidWrite(double output) {
          ElevatorSub.set(output, true);
          SmartDashboard.putNumber("Output", output);
        }
      }, 0.01);
  private double setpoint;

  public ElevateToHeight(double height) {
    requires(Robot.elevatorSub);
    setpoint = height;

  }

  protected void initialize() {
    LEDSub.actionsInProgress++;
    ElevateToHeight.isElevating = true;
    if (pid == null)
      return;
    if (setpoint > ElevatorSub.getHeight()) {
      pid.setPID(RobotSettings.ELEVATOR_UP_KP, RobotSettings.ELEVATOR_UP_KI, RobotSettings.ELEVATOR_UP_KD);
    } else {
      pid.setPID(RobotSettings.ELEVATOR_DOWN_KP, RobotSettings.ELEVATOR_DOWN_KI, RobotSettings.ELEVATOR_DOWN_KD);
    }
    pid.setSetpoint(setpoint);
    pid.setOutputRange(-RobotSettings.ELEVATOR_SPEED_MULTIPLYER, RobotSettings.ELEVATOR_SPEED_MULTIPLYER);
    pid.enable();
  }

  protected boolean isFinished() {
    return Math.abs(ElevatorSub.getHeight() - setpoint) < 1.5 && ElevatorSub.getRate() < 1.5;
  }

  protected void end() {
    LEDSub.actionsInProgress--;
    ElevateToHeight.isElevating = false;
    if (pid != null) {
      pid.disable();
    }
    ElevatorSub.set(0);
  }
  @Override
  protected void interrupted() {
    end();
  }
}
