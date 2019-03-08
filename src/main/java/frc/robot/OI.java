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

  /*-----Driver Stick-----*/
  private static Button vision;
  private static Button toggleBack;
  private static Button toggleFront;
  private static Button tapLeft;
  private static Button tapRight;
  private static Button driverPickup;
  private static Button testButton;

  /*-----Hatch Stick-----*/
  private static Button shootHatch;
  private static Button pickupHatch;
  private static Button extendHatch;
  private static Button retractHatch;
  private static Button grabHatch;
  private static Button releaseHatch;
  private static Button hatchL3;
  private static Button hatchL2;
  private static Button hatchCargo;
  private static Button hatchL1;

  /*-----Cargo Stick-----*/
  private static Button shootCargo;
  private static Button pickupCargo;
  private static Button spinIntakeIn;
  private static Button spinIntakeOut;
  private static Button lowerIntake;
  private static Button raiseIntake;
  private static Button cargoL3;
  private static Button cargoL2;
  private static Button cargoShip;
  private static Button cargoL1;

  public void setupOI() {
    /*-----Driver Stick-----*/
    vision = new JoystickButton(RobotIO.driverStick, 1);
    toggleBack = new JoystickButton(RobotIO.driverStick, 2);
    toggleFront = new JoystickButton(RobotIO.driverStick, 3);
    tapLeft = new JoystickButton(RobotIO.driverStick, 4);
    tapRight = new JoystickButton(RobotIO.driverStick, 5);
    driverPickup = new JoystickButton(RobotIO.driverStick, 8);
    testButton = new JoystickButton(RobotIO.driverStick, 6);
    vision.whenPressed(new GoToTarget());
    vision.whenReleased(new EnableVision(false));
    vision.whenReleased(new JoystickDrive());
    toggleBack.whenPressed(new ClimberBackToggle());
    toggleFront.whenPressed(new ClimberFrontToggle());
    tapLeft.whenPressed(new RotateAngle(-2));
    tapRight.whenPressed(new RotateAngle(2));

    testButton.whenPressed(new DriveUsingTrajectory("L2CargoShipSideLeft"));
    driverPickup.whenPressed(new PickupCargo());
    driverPickup.whileHeld(new ElevateToHeight(0));
    driverPickup.whenReleased(new CargoForwardUntilClear());
    driverPickup.whenReleased(new IntakeDoNothing());
    driverPickup.whenReleased(new RaiseIntake());
    /*-----Hatch Stick-----*/
    shootHatch = new JoystickButton(RobotIO.hatchStick, 1);
    pickupHatch = new JoystickButton(RobotIO.hatchStick, 2);
    retractHatch = new JoystickButton(RobotIO.hatchStick, 3);
    grabHatch = new JoystickButton(RobotIO.hatchStick, 4);
    extendHatch = new JoystickButton(RobotIO.hatchStick, 5);
    releaseHatch = new JoystickButton(RobotIO.hatchStick, 6);

    hatchL3 = new JoystickButton(RobotIO.hatchStick, 8);
    hatchL2 = new JoystickButton(RobotIO.hatchStick, 10);
    hatchCargo = new JoystickButton(RobotIO.hatchStick, 11);
    hatchL1 = new JoystickButton(RobotIO.hatchStick, 12);

    shootHatch.whenPressed(new ShootHatch());
    pickupHatch.whenPressed(new PickupHatch());
    extendHatch.whenPressed(new ExtendHatch());
    retractHatch.whenPressed(new RetractHatch());
    grabHatch.whenPressed(new GrabHatch());
    releaseHatch.whenPressed(new ReleaseHatch());

    hatchL3.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_3));
    hatchL3.whenPressed(new SetRocketOrCargo(true));
    hatchL2.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_2));
    hatchL2.whenPressed(new SetRocketOrCargo(true));
    hatchCargo.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_1));
    hatchCargo.whenPressed(new SetRocketOrCargo(false));
    hatchL1.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_1));
    hatchL1.whenPressed(new SetRocketOrCargo(true));

    /*-----Cargo Stick-----*/
    shootCargo = new JoystickButton(RobotIO.cargoStick, 1);
    pickupCargo = new JoystickButton(RobotIO.cargoStick, 2);
    spinIntakeOut = new JoystickButton(RobotIO.cargoStick, 3);
    lowerIntake = new JoystickButton(RobotIO.cargoStick, 4);
    spinIntakeIn = new JoystickButton(RobotIO.cargoStick, 5);
    raiseIntake = new JoystickButton(RobotIO.cargoStick, 6);
    cargoL3 = new JoystickButton(RobotIO.cargoStick, 8);
    cargoL2 = new JoystickButton(RobotIO.cargoStick, 10);
    cargoShip = new JoystickButton(RobotIO.cargoStick, 11);
    cargoL1 = new JoystickButton(RobotIO.cargoStick, 12);

    shootCargo.whileHeld(new ShootCargo(RobotSettings.CARGO_SHOOT_SPEED, false));
    pickupCargo.whenPressed(new PickupCargo());
    pickupCargo.whileHeld(new ElevateToHeight(0));
    pickupCargo.whenReleased(new CargoForwardUntilClear());
    pickupCargo.whenReleased(new IntakeDoNothing());
    pickupCargo.whenReleased(new RaiseIntake());

    spinIntakeOut.whileHeld(new SpinIntake(-1.0));
    spinIntakeOut.whenReleased(new IntakeDoNothing());
    spinIntakeIn.whileHeld(new SpinIntake(1.0));
    spinIntakeIn.whenReleased(new IntakeDoNothing());
    lowerIntake.whenPressed(new LowerIntake());
    raiseIntake.whenPressed(new RaiseIntake());
    
    cargoL3.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_3 + RobotSettings.ELEVATOR_CARGO_OFFSET));
    cargoL3.whenPressed(new SetRocketOrCargo(true));
    cargoL2.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_2 + RobotSettings.ELEVATOR_CARGO_OFFSET));
    cargoL2.whenPressed(new SetRocketOrCargo(true));
    cargoShip.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_CARGO_SHIP));
    cargoShip.whenPressed(new SetRocketOrCargo(false));
    cargoL1.whenPressed(new ElevateToHeight(RobotSettings.ELEVATOR_HATCH_LEVEL_1 + RobotSettings.ELEVATOR_CARGO_OFFSET));
    cargoL1.whenPressed(new SetRocketOrCargo(true));

  }
}
