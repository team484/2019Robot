/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class RobotIO {
    public static CANSparkMax leftMotor1;
    public static CANSparkMax leftMotor2;
    public static CANSparkMax leftMotor3;
    public static CANSparkMax rightMotor1;
    public static CANSparkMax rightMotor2;
    public static CANSparkMax rightMotor3;
    public static CANSparkMax elevatorMotorLeft;
    public static CANSparkMax elevatorMotorRight;

    
    public static WPI_TalonSRX cargoController;
    public static DoubleSolenoid topSolenoid;
    public static DoubleSolenoid bottomSolenoid;

    public static WPI_TalonSRX wheelController;
    public static DoubleSolenoid armSolenoid;
    public static AnalogInput cargoIrFinder;


    public static DifferentialDrive diffDrive;

    public static Joystick driverStick;
    public static Joystick operatorStick;
    public RobotIO() {
        leftMotor1 = new CANSparkMax(RobotSettings.LEFT_MOTOR_1_ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(RobotSettings.LEFT_MOTOR_2_ID, MotorType.kBrushless);
        leftMotor3 = new CANSparkMax(RobotSettings.LEFT_MOTOR_3_ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_1_ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_2_ID, MotorType.kBrushless);
        rightMotor3 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_3_ID, MotorType.kBrushless);
        
        
        
        
        
        
        
        
        cargoIrFinder = new AnalogInput(RobotSettings.IR_RANGE_FINDER);
        cargoController = new WPI_TalonSRX(RobotSettings.CARGO_CONTROLLER_ID);
        topSolenoid = new DoubleSolenoid(RobotSettings.TOP_SOLENOID_PORT_1, RobotSettings.TOP_SOLENOID_PORT_2);
        bottomSolenoid = new DoubleSolenoid(RobotSettings.BOTTOM_SOLENOID_PORT_1, RobotSettings.BOTTOM_SOLENOID_PORT_2);
        elevatorMotorLeft = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
        elevatorMotorRight = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);
    
        wheelController = new WPI_TalonSRX(RobotSettings.WHEEL_CONTROLLER_ID);
        armSolenoid = new DoubleSolenoid(RobotSettings.ARM_SOLENOID_PORT_1, RobotSettings.ARM_SOLENOID_PORT_2);


        leftMotor2.follow(leftMotor1);
        leftMotor3.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        rightMotor3.follow(rightMotor1);

        diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);

        driverStick = new Joystick(RobotSettings.DRIVER_STICK_PORT);
        operatorStick = new Joystick(RobotSettings.OPERATOR_STICK_PORT);
    }
}
