/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotIO;
import frc.robot.commands.LEDCommand;

/**
 * Add your docs here.
 */
public class LEDSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LEDCommand());
  }

  public static void setRGB(int r, int g, int b) { //0-255
    RobotIO.led_r.setRaw(r);
  }

  private static long i = 0;
  public static int actionsInProgress = 0;
  public static void defaultColors() {
    i++;
    int r = 255;
    int g = 0;
    int b = 0;
    if (actionsInProgress > 0) {
      r = 0;
      b = 255;
    }

    if (System.currentTimeMillis() - Robot.matchStartTime < 120*1000 && i % 50 < 25) {
      r = 0;
      g = 0;
      b = 0;
    }
    setRGB(r, g, b);
  }
}
