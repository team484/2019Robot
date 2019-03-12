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
import frc.robot.RobotIO;
import frc.robot.subsystems.ElevatorSub;

/**
 * Uses the operator joystick to control the elevator height
 */
public class JoystickElevator extends Command {
  PIDSource pidSource = new PIDSource() {

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {

    }

    @Override
    public double pidGet() {
      return ElevatorSub.getHeight();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return PIDSourceType.kDisplacement;
    }
  };

  PIDOutput pidOutput = new PIDOutput() {

    @Override
    public void pidWrite(double output) {
      if (pid.getSetpoint() > 2) {
        ElevatorSub.set(output);
      } else {
        ElevatorSub.set(0);
      }
    }
  };

  PIDController pid = new PIDController(0.02, 0.0, 0.0, pidSource, pidOutput, 0.1);

  private int commandState = 0; // 0 = JoystickCtrl 1 = No Joystick, but not stationary 2 = Stationary/PID
                                // control
  private double lastRate = 0;

  public JoystickElevator() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.elevatorSub);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    commandState = 0;
    lastRate = ElevatorSub.getRate();
    pid.setOutputRange(-0.3, 0.3);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("ElevState", commandState);
    double newRate = ElevatorSub.getRate();
    double operatorInput1 = RobotIO.hatchStick.getY();
    double operatorInput2 = RobotIO.cargoStick.getY();
    double operatorInput = Math.abs(operatorInput1) > Math.abs(operatorInput2) ? operatorInput1 : operatorInput2;
    if (Math.abs(operatorInput) < 0.1) {
      operatorInput = 0;
    }
    if (Math.abs(operatorInput) > 0.08) {
      commandState = 0;
      pid.disable();
    }

    switch (commandState) {
    case 0:
      pid.disable();
      ElevatorSub.set(operatorInput, true);
      if (Math.abs(operatorInput) < 0.08) {
        commandState = 1;
      }
      break;

    case 1:
      ElevatorSub.set(operatorInput, true);
      if (Math.abs(newRate) < 1) {
        pid.setSetpoint(ElevatorSub.getHeight());
        pid.reset();
        pid.enable();
        commandState = 2;
        break;
      }
      if (newRate >= 0 && lastRate <= 0) {
        pid.setSetpoint(ElevatorSub.getHeight());
        pid.reset();
        pid.enable();
        commandState = 2;
        break;
      }
      if (newRate <= 0 && lastRate >= 0) {
        pid.setSetpoint(ElevatorSub.getHeight());
        pid.reset();
        pid.enable();
        commandState = 2;
        break;
      }
    case 2:
      if (pid.getSetpoint() < 2) {
        if (pid.isEnabled()) {
          pid.disable();
        }
      }
    default:
      break;
    }
    lastRate = newRate;

  }

  // This command is never finished
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after command is interrupted
  @Override
  protected void end() {
    ElevatorSub.set(0);
    pid.disable();
  }
  @Override
  protected void interrupted() {
    end();
  }

}
