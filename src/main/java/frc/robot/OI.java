/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.*;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  private static Button pickupCargo;
  private static Button shootCargo;
  private static Button extendHatch;
  private static Button grabHatch;
  private static Button raiseIntake;
  private static Button releaseHatch;
  private static Button retractHatch;
  private static Button lowerBack;
  private static Button raiseBack;
  private static Button lowerFront;
  private static Button raiseFront;
  private static Button hatchL2;

  public void setupOI() {
    shootCargo = new JoystickButton(RobotIO.operatorStick, 1);
    pickupCargo = new JoystickButton(RobotIO.operatorStick, 2);
    extendHatch = new JoystickButton(RobotIO.operatorStick, 3);
    retractHatch = new JoystickButton(RobotIO.operatorStick, 4);
    grabHatch = new JoystickButton(RobotIO.operatorStick, 5);
    releaseHatch = new JoystickButton(RobotIO.operatorStick, 6);
    raiseIntake = new JoystickButton(RobotIO.operatorStick, 7);
    lowerBack = new JoystickButton(RobotIO.driverStick, 2);
    raiseBack = new JoystickButton(RobotIO.driverStick, 3);
    lowerFront = new JoystickButton(RobotIO.driverStick, 5);
    raiseFront = new JoystickButton(RobotIO.driverStick, 4);
    hatchL2 = new JoystickButton(RobotIO.operatorStick, 10);

    pickupCargo.whenPressed(new PickupCargo());
    pickupCargo.whenReleased(new CargoForwardUntilClear());
    pickupCargo.whenReleased(new IntakeDoNothing());
    pickupCargo.whenReleased(new RaiseIntake());
    shootCargo.whileHeld(new ShootCargo(RobotSettings.CARGO_SHOOT_SPEED, false));
    extendHatch.whenPressed(new ExtendHatch());
    grabHatch.whenPressed(new GrabHatch());
    raiseIntake.whenPressed(new RaiseIntake());
    releaseHatch.whenPressed(new ReleaseHatch());
    retractHatch.whenPressed(new RetractHatch());
    lowerBack.whenPressed(new ClimberBackDown());
    raiseBack.whenPressed(new ClimberBackUp());
    lowerFront.whenPressed(new ClimberFrontDown());
    raiseFront.whenPressed(new ClimberFrontUp());
    hatchL2.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_2));
  }
}
