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

/**
 * Raises/lowers the elevator to the specified height
 */
public class ElevateToHeight extends Command {

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
  private boolean isGoingUp = false;

  public ElevateToHeight(double height) {
    requires(Robot.elevatorSub);
    setpoint = height;

  }

  protected void initialize() {
    if (pid == null)
      return;
    if (setpoint > ElevatorSub.getHeight()) {
      isGoingUp = true;
      pid.setPID(RobotSettings.ELEVATOR_UP_KP, RobotSettings.ELEVATOR_UP_KI, RobotSettings.ELEVATOR_UP_KD);
    } else {
      isGoingUp = false;
      pid.setPID(RobotSettings.ELEVATOR_DOWN_KP, RobotSettings.ELEVATOR_DOWN_KI, RobotSettings.ELEVATOR_DOWN_KD);
    }
    pid.setSetpoint(setpoint);
    pid.setOutputRange(-RobotSettings.ELEVATOR_SPEED_MULTIPLYER, RobotSettings.ELEVATOR_SPEED_MULTIPLYER);
    pid.enable();
  }

  protected boolean isFinished() {
    return (((ElevatorSub.getHeight() >= setpoint && isGoingUp) || (ElevatorSub.getHeight() <= setpoint && !isGoingUp))
        && Math.abs(ElevatorSub.getRate()) < 0);
  }

  protected void end() {
    if (pid != null) {
      pid.disable();
    }
    ElevatorSub.set(0);
  }
}
