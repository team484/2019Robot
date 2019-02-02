/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * All robot settings, including ports, IDs, constants, and all magic numbers
 */
public class RobotSettings {

    /*-----CAN IDs-----*/
    public static final int LEFT_MOTOR_1_ID = 1;
    public static final int LEFT_MOTOR_2_ID = 2;
    public static final int LEFT_MOTOR_3_ID = 3;
    public static final int RIGHT_MOTOR_1_ID = 4;
    public static final int RIGHT_MOTOR_2_ID = 5;
    public static final int RIGHT_MOTOR_3_ID = 6;
    public static final int ELEVATOR_MOTOR_LEFT_ID = 7;
    public static final int ELEVATOR_MOTOR_RIGHT_ID = 8;
    public static final int INTAKE_MOTOR_ID = 9;
    public static final int CARGO_MOTOR_ID = 10;

    /*-----Analog Ports-----*/
    public static final int CARGO_SENSOR_PORT = 0;
    
    /*-----Joystick Ports-----*/
    public static final int DRIVER_STICK_PORT = 0;
    public static final int OPERATOR_STICK_PORT = 9;

    /*-----PCM Ports-----*/
    public static final int TOP_SOLENOID_PORT_1 = 0;
    public static final int TOP_SOLENOID_PORT_2 = 1;
    public static final int BOTTOM_SOLENOID_PORT_1 = 2;
    public static final int BOTTOM_SOLENOID_PORT_2 = 3;
    public static final int ARM_SOLENOID_PORT_1 = 4;
    public static final int ARM_SOLENOID_PORT_2 = 5;

    /*-----Cargo Vars-----*/
    public static final double CARGO_SHOOT_SPEED = 0.5;
    public static final double CARGO_SENSOR_VOLTAGE = 2.0;

    /*-----Drive Vars-----*/

    /*-----Elevator Vars-----*/
    public static final double ELEVATOR_SPEED_MULTIPLYER = 1.0;
    public static final double ELEVATOR_UP_KP = 0;
    public static final double ELEVATOR_UP_KI = 0;
    public static final double ELEVATOR_UP_KD = 0;

    /*-----Hatch Vars-----*/

    /*-----Intake Vars-----*/
    public static final double INTAKE_MOTOR_SPEED = 1.0;

}