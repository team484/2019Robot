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
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Initializes all robot Input/Output object.
 * (e.g. speed controllers, joysticks, etc.)
 */
public class RobotIO {
    public static ArrayList<String> errors = new ArrayList<>();

    /*-----Cargo-----*/
    public static WPI_TalonSRX cargoMotor;
    public static AnalogInput cargoSensor;

    /*-----Climber-----*/
    public static DoubleSolenoid climberBackSolenoid;
    public static DoubleSolenoid climberFrontSolenoid;

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
    public static DigitalInput elevatorDownSwitch;

    /*-----Hatch-----*/
    public static DoubleSolenoid topHatchSolenoid;
    public static DoubleSolenoid bottomHatchSolenoid;

    /*-----Intake-----*/
    public static WPI_TalonSRX intakeMotor;
    public static DoubleSolenoid intakeSolenoid;
    public static AnalogInput intakeSensor;

    /*-----Misc-----*/
    public static Joystick driverStick;
    public static Joystick hatchStick;
    public static Joystick cargoStick;
    public static PigeonIMU imu;
    public static Compressor compressor;
    public static PowerDistributionPanel pdp;

    public RobotIO() {
        /*-----Cargo-----*/
        cargoSensor = new AnalogInput(RobotSettings.CARGO_SHOOTER_SENSOR_PORT);
        cargoMotor = new WPI_TalonSRX(RobotSettings.INTAKE_MOTOR_ID);
        cargoMotor.configVoltageCompSaturation(12);
        cargoMotor.enableVoltageCompensation(true);
        cargoMotor.setInverted(true);

        /*-----Climber-----*/
        climberBackSolenoid = new DoubleSolenoid(1, RobotSettings.CLIMBER_SOLENOID_PORT_1,
                RobotSettings.CLIMBER_SOLENOID_PORT_2);
        climberFrontSolenoid = new DoubleSolenoid(RobotSettings.CLIMBER_SOLENOID_FRONT_PORT_1,
                RobotSettings.CLIMBER_SOLENOID_FRONT_PORT_2);
        climberBackSolenoid.set(Value.kReverse);
        climberFrontSolenoid.set(Value.kReverse);

        /*-----Drivetrain-----*/
        leftMotor1 = new CANSparkMax(RobotSettings.LEFT_MOTOR_1_ID, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(RobotSettings.LEFT_MOTOR_2_ID, MotorType.kBrushless);
        leftMotor3 = new CANSparkMax(RobotSettings.LEFT_MOTOR_3_ID, MotorType.kBrushless);
        rightMotor1 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_1_ID, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_2_ID, MotorType.kBrushless);
        rightMotor3 = new CANSparkMax(RobotSettings.RIGHT_MOTOR_3_ID, MotorType.kBrushless);

        if (leftMotor1.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with left drive motor 1");
        }
        if (leftMotor2.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with left drive motor 2");
        }
        if (leftMotor3.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with left drive motor 3");
        }
        if (rightMotor1.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with right drive motor 1");
        }
        if (rightMotor2.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with right drive motor 2");
        }
        if (rightMotor3.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with right drive motor 3");
        }
        leftMotor2.follow(leftMotor1);
        leftMotor3.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);
        rightMotor3.follow(rightMotor1);

        leftMotor1.setInverted(RobotSettings.INVERT_DRIVE);
        leftMotor2.setInverted(RobotSettings.INVERT_DRIVE);
        leftMotor3.setInverted(RobotSettings.INVERT_DRIVE);
        rightMotor1.setInverted(RobotSettings.INVERT_DRIVE);
        rightMotor2.setInverted(RobotSettings.INVERT_DRIVE);
        rightMotor3.setInverted(RobotSettings.INVERT_DRIVE);
        diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);

        leftEncoder = new Encoder(RobotSettings.LEFT_ENCODER_A_PORT, RobotSettings.LEFT_ENCODER_B_PORT);
        rightEncoder = new Encoder(RobotSettings.RIGHT_ENCODER_A_PORT, RobotSettings.RIGHT_ENCODER_B_PORT);

        leftEncoder.setDistancePerPulse(RobotSettings.DRIVE_ENCODER_DPP);
        rightEncoder.setDistancePerPulse(-RobotSettings.DRIVE_ENCODER_DPP);

        /*-----Elevator-----*/
        elevatorMotorLeft = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_LEFT_ID, MotorType.kBrushless);
        elevatorMotorLeft.setSmartCurrentLimit(80);
        elevatorMotorLeft.setOpenLoopRampRate(1.0);
        elevatorMotorLeft.setClosedLoopRampRate(1.0);
        elevatorMotorLeft.enableVoltageCompensation(10);
        if (elevatorMotorLeft.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with left elevator motor");
        }

        elevatorMotorRight = new CANSparkMax(RobotSettings.ELEVATOR_MOTOR_RIGHT_ID, MotorType.kBrushless);
        elevatorMotorRight.setSmartCurrentLimit(80);
        elevatorMotorRight.setOpenLoopRampRate(1.0);
        elevatorMotorRight.setClosedLoopRampRate(1.0);
        elevatorMotorRight.enableVoltageCompensation(10);
        elevatorMotorRight.getEncoder().setPosition(0);
        if (elevatorMotorRight.setEncPosition(0) != CANError.kOK) {
            errors.add("Error with right elevator motor");
        }

        elevatorDownSwitch = new DigitalInput(RobotSettings.ELEVATOR_DOWN_SWITCH_PORT);

        /*-----Hatch-----*/
        topHatchSolenoid = new DoubleSolenoid(RobotSettings.TOP_SOLENOID_PORT_1, RobotSettings.TOP_SOLENOID_PORT_2);
        bottomHatchSolenoid = new DoubleSolenoid(RobotSettings.BOTTOM_SOLENOID_PORT_1,
                RobotSettings.BOTTOM_SOLENOID_PORT_2);
        topHatchSolenoid.set(Value.kReverse);
        bottomHatchSolenoid.set(Value.kReverse);
        /*-----Intake-----*/
        intakeMotor = new WPI_TalonSRX(RobotSettings.CARGO_MOTOR_ID);
        intakeMotor.configVoltageCompSaturation(12);
        intakeMotor.enableVoltageCompensation(true);
        intakeSolenoid = new DoubleSolenoid(1, RobotSettings.ARM_SOLENOID_PORT_1, RobotSettings.ARM_SOLENOID_PORT_2);
        intakeSensor = new AnalogInput(RobotSettings.CARGO_INTAKE_SENSOR_PORT);
        intakeSolenoid.set(Value.kForward);

        /*-----Misc-----*/
        driverStick = new Joystick(RobotSettings.DRIVER_STICK_PORT);
        hatchStick = new Joystick(RobotSettings.HATCH_STICK_PORT);
        cargoStick = new Joystick(RobotSettings.CARGO_STICK_PORT);
        imu = new PigeonIMU(cargoMotor);
        compressor = new Compressor();
        pdp = new PowerDistributionPanel();
        if (pdp.getTemperature() < 0) {
            errors.add("PDP error");
        }
        SmartDashboard.putStringArray("ErrorLog", errors.toArray(new String[errors.size()]));
        for (int i = 0; i < 5; i++) {
            String text = errors.size() > i ? errors.get(i) : "";
            SmartDashboard.putString("err" + i, text);
        }
        SmartDashboard.putNumber("Errors", errors.size());
    }
}
