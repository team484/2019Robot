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



  public void setupOI() {
    
    pickupCargo = new JoystickButton(RobotIO.operatorStick, 0);
    shootCargo = new JoystickButton(RobotIO.operatorStick, 0);
    extendHatch = new JoystickButton(RobotIO.operatorStick, 0);
    grabHatch = new JoystickButton(RobotIO.operatorStick, 0);
    raiseIntake = new JoystickButton(RobotIO.operatorStick, 0);
    releaseHatch = new JoystickButton(RobotIO.operatorStick, 0);
    retractHatch = new JoystickButton(RobotIO.operatorStick, 0);

    pickupCargo.whenPressed(new PickupCargo());
    shootCargo.whenPressed(new ShootCargo());
    extendHatch.whenPressed(new ExtendHatch());
    grabHatch.whenPressed(new GrabHatch());
    raiseIntake.whenPressed(new RaiseIntake());
    releaseHatch.whenPressed(new ReleaseHatch());
    retractHatch.whenPressed(new RetractHatch());



  }
}
