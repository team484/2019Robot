/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CargoSub;

/**
 * Spins the cargo shooter
 */
public class ShootCargo extends Command {
  private double speed;
  private boolean ramp;
  private double i = 0.0;

  public ShootCargo(double speed, boolean ramp) {
    requires(Robot.cargoSub);
    this.speed = speed;
    this.ramp = ramp;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    i = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (ElevateToHeight.isElevating) {
      CargoSub.set(0);
      return;
    }
    if (i < speed && ramp) {
      CargoSub.set(i);
      i += 0.025;
      return;
    }
    CargoSub.set(speed);
  }

  // This command never finishes. Interrupt it to end it.
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after command is interrupted
  @Override
  protected void end() {
    CargoSub.set(0);
  }
  @Override
  protected void interrupted() {
    end();
  }

}
