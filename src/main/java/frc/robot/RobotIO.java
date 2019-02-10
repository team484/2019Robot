/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class RobotIO {
    public static ArrayList<String> errors = new ArrayList<>();
    /*-----Cargo-----*/
    public static WPI_TalonSRX cargoMotor;
    public static AnalogInput cargoSensor;

    /*-----Drivetrain-----*/
    public static CANSparkMax leftMotor1;
    public static CANSparkMax leftMotor2;
    public static CANSparkMax leftMotor3;
    public static CANSparkMax rightMotor1;
    public static CANSparkMax rightMotor2;
    public static CANSparkMax rightMotor3;
    public static DifferentialDrive diffDrive;
    public static Encoder leftEncoder;
    public static Encoder rightEncoder;

    /*-----Elevator-----*/
    public static CANSparkMax elevatorMotorLeft;
    public static CANSparkMax elevatorMotorRight;

    /*-----Hatch-----*/
    public static DoubleSolenoid topHatchSolenoid;
    public static DoubleSolenoid bottomHatchSolenoid;

    /*-----Intake-----*/
    public static WPI_TalonSRX intakeMotor;
    public static DoubleSolenoid intakeSolenoid;

    /*-----Misc-----*/
    public static Joystick driverStick;
    public static Joystick operatorStick;
    public static PigeonIMU imu;
    public static Compressor compressor;
    public static PowerDistributionPanel pdp;

    /*-----Climber-----*/
    public static WPI_TalonSRX leftClimberMotor;
    public static WPI_TalonSRX rightClimberMotor;
    public static DoubleSolenoid climberPistonSolenoid;

    public RobotIO() {
        /*-----Cargo-----*/
        cargoSensor = new AnalogInput(RobotSettings.CARGO_SENSOR_PORT);
        cargoMotor = new WPI_TalonSRX(RobotSettings.INTAKE_MOTOR_ID);
        cargoMotor.setInverted(true);

        /*-----Drivetrain-----*/
        try {
            leftMotor1 = new CANSparkMax(RobotSettings.LEFT_MOTOR_1_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to left drive motor 1");
        }
        try {
            leftMotor2 = new CANSparkMax(RobotSettings.LEFT_MOTOR_2_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to left drive motor 2");
        }
        try {
            leftMotor3 = new CANSparkMax(RobotSettings.LEFT_MOTOR_3_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to left drive motor 3");
        }
        try {
            rightMotor1 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_1_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to right drive motor 1");
        }
        try {
            rightMotor2 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_2_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to right drive motor 2");
        }
        try {
            rightMotor3 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_3_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to right drive motor 3");
        }
        if (leftMotor1 == null) {
            if (leftMotor2 != null) {
                leftMotor1 = leftMotor2;
            } else if (leftMotor3 != null) {
                leftMotor1 = leftMotor3;
                leftMotor2 = leftMotor3;
            }
        }
        if (rightMotor1 == null) {
            if (rightMotor2 != null) {
                rightMotor1 = rightMotor2;
            } else if (rightMotor3 != null) {
                rightMotor1 = rightMotor3;
                rightMotor2 = rightMotor3;
            }
        }
        if (leftMotor1 != leftMotor2) {
            leftMotor2.follow(leftMotor1);
        }
        if (leftMotor1 != leftMotor3) {
            leftMotor3.follow(leftMotor1);
        }
        if (rightMotor1 != rightMotor2) {
            rightMotor2.follow(rightMotor1);
        }
        if (rightMotor1 != rightMotor3) {
            rightMotor3.follow(rightMotor1);
        }
        if (leftMotor1 != null) {
            leftMotor1.setInverted(RobotSettings.INVERT_DRIVE);
            leftMotor2.setInverted(RobotSettings.INVERT_DRIVE);
            leftMotor3.setInverted(RobotSettings.INVERT_DRIVE);
        }
        if (rightMotor1 != null) {
            rightMotor1.setInverted(RobotSettings.INVERT_DRIVE);
            rightMotor2.setInverted(RobotSettings.INVERT_DRIVE);
            rightMotor3.setInverted(RobotSettings.INVERT_DRIVE);
        }
        if (leftMotor1 != null && rightMotor1 != null) {
            diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);
        }
        leftEncoder = new Encoder(RobotSettings.LEFT_ENCODER_A_PORT, RobotSettings.LEFT_ENCODER_B_PORT);
        rightEncoder = new Encoder(RobotSettings.RIGHT_ENCODER_A_PORT, RobotSettings.RIGHT_ENCODER_B_PORT);

        leftEncoder.setDistancePerPulse(RobotSettings.DRIVE_ENCODER_DPP);
        rightEncoder.setDistancePerPulse(-RobotSettings.DRIVE_ENCODER_DPP);
        /*-----Elevator-----*/
        try {
            elevatorMotorLeft = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_LEFT_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to left elevator motor");
        }
        try {
            elevatorMotorRight = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
        } catch (Exception e) {
            e.printStackTrace();
            errors.add("Cannot connect to right elevator motor");
        }
        /*-----Hatch-----*/
        topHatchSolenoid = new DoubleSolenoid(RobotSettings.TOP_SOLENOID_PORT_1, RobotSettings.TOP_SOLENOID_PORT_2);
        bottomHatchSolenoid = new DoubleSolenoid(RobotSettings.BOTTOM_SOLENOID_PORT_1,
                RobotSettings.BOTTOM_SOLENOID_PORT_2);

        /*-----Intake-----*/
        intakeMotor = new WPI_TalonSRX(RobotSettings.CARGO_MOTOR_ID);
        intakeSolenoid = new DoubleSolenoid(1,RobotSettings.ARM_SOLENOID_PORT_1, RobotSettings.ARM_SOLENOID_PORT_2);

        /*-----Climber-----*/
        leftClimberMotor = new WPI_TalonSRX(RobotSettings.CLIMBER_MOTOR_LEFT);
        rightClimberMotor = new WPI_TalonSRX(RobotSettings.CLIMBER_MOTOR_RIGHT);
        climberPistonSolenoid = new DoubleSolenoid(1,RobotSettings.CLIMBER_SOLENOID_PORT_1, RobotSettings.CLIMBER_SOLENOID_PORT_2);


        /*-----Misc-----*/
        driverStick = new Joystick(RobotSettings.DRIVER_STICK_PORT);
        operatorStick = new Joystick(RobotSettings.OPERATOR_STICK_PORT);
        imu = new PigeonIMU(cargoMotor);
        compressor = new Compressor();
        pdp = new PowerDistributionPanel();
    }
}
