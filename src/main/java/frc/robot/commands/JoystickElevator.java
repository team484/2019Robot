/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.subsystems.ElevatorSub;

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
      ElevatorSub.set(output);
    }
  };

  PIDController pid = new PIDController(0.1, 0.0, 0.0, pidSource, pidOutput, 0.1);

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
    pid.setOutputRange(-2.0, 2.0);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("ElevState", commandState);
    double newRate = ElevatorSub.getRate();
    double operatorInput = RobotIO.operatorStick.getY();
    if (Math.abs(operatorInput) > 0.06 || DriverStation.getInstance().isDisabled()) {
      commandState = 0;
      pid.disable();
    }

    switch (commandState) {
    case 0:
      ElevatorSub.set(operatorInput * 8.0);
      if (Math.abs(operatorInput) < 0.06) {
        commandState = 1;
      }
      break;

    case 1:
      if (newRate == 0) {
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
        ElevatorSub.set(0);
      }
    default:
      break;
    }
    lastRate = newRate;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
