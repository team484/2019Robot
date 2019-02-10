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
    public static final int CLIMBER_MOTOR_LEFT = 11;
    public static final int CLIMBER_MOTOR_RIGHT = 12;

    /*-----Analog Ports-----*/
    public static final int CARGO_SENSOR_PORT = 0;

    /*-----Digital Ports-----*/
    public static final int LEFT_ENCODER_A_PORT = 0;
    public static final int LEFT_ENCODER_B_PORT = 1;
    public static final int RIGHT_ENCODER_A_PORT = 2;
    public static final int RIGHT_ENCODER_B_PORT = 3;
    
    
    /*-----Joystick Ports-----*/
    public static final int DRIVER_STICK_PORT = 0;
    public static final int OPERATOR_STICK_PORT = 1;

    /*-----PCM Ports-----*/
    public static final int TOP_SOLENOID_PORT_1 = 2;
    public static final int TOP_SOLENOID_PORT_2 = 3;
    public static final int BOTTOM_SOLENOID_PORT_1 = 1;
    public static final int BOTTOM_SOLENOID_PORT_2 = 0;

    public static final int ARM_SOLENOID_PORT_1 = 0;
    public static final int ARM_SOLENOID_PORT_2 = 1;
    public static final int CLIMBER_SOLENOID_PORT_1 = 2;
    public static final int CLIMBER_SOLENOID_PORT_2 = 3;

    /*-----Cargo Vars-----*/
    public static final double CARGO_SHOOT_SPEED = 0.8;
    public static final double CARGO_SENSOR_VOLTAGE = 2.0;

    /*-----Drive Vars-----*/
    public static final double DRIVE_ENCODER_DPP = 0.04908738521;
    public static final double DRIVE_MOTOR_ENC_DPP = 1.1170107213;
    public static final boolean INVERT_DRIVE = false;

    public static final double TIME_STEP = 0.02;
	public static final double MAX_VELOCITY = 3.0; //m/s
	public static final double MAX_ACCELERATION = 2.5; //m/s^2
	public static final double MAX_JERK = 60; //m/s^3
	public static final double WHEELBASE_WIDTH = 27;
	public static final String SAVE_DIR_TRAJECTORIES = "/home/lvuser/trajectories/";
	public static final int LEFT_ENC_TIC_PER_ROT = (int) (1000.0 * 4.0 * Math.PI); // Encoder ticks per wheel rotation
	public static final int RIGHT_ENC_TIC_PER_ROT = (int) (1000.0 * 4.0 * Math.PI); //Encoder ticks per wheel rotation
	public static final double WHEEL_DIAMETER = 4.0 * 0.0254; //Wheel diameter (in) * m/in
    public static final double VOLTAGE_TARGET = 8.3;

    public static final double ROTATE_ANGLE_KP = 0.06;
	public static final double ROTATE_ANGLE_KI = 0;
	public static final double ROTATE_ANGLE_KD = 0.6;
    public static final double ROTATE_PID_UPDATE_RATE = 0.01;
    public static final double ROTATE_PID_TOLERANCE = 0;
    
    public static final double DRIVE_PID_UPDATE_RATE = 0.01;
    public static final double MAINTAIN_ANGLE_KP = 0.05;
	public static final double MAINTAIN_ANGLE_KI = 0.001;
	public static final double MAINTAIN_ANGLE_KD = 0.0;
	public static final double DRIVE_PID_TOLERANCE = 8;
    public static final double DRIVE_PID_RATE_TOLERANCE = 0.5;
    public static final double DRIVE_DISTANCE_KP = 0.05;
	public static final double DRIVE_DISTANCE_KI = 0;
	public static final double DRIVE_DISTANCE_KD = 0.75;

    // The first argument is the proportional gain. Usually this will be quite high
	// The second argument is the integral gain. This is unused for motion profiling
	// The third argument is the derivative gain. Tweak this if unhappy with the tracking of the trajectory
	// The fourth argument is the velocity ratio. This is 1 over the maximum velocity provided in the 
	// trajectory configuration (it translates m/s to a -1 to 1 scale that the motors can read)
	// The fifth argument is the acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
	public static final double KP = 1.0;
	public static final double KI = 0.0;
	public static final double KD = 0.2;
	public static final double VELOCITY_RATIO = 1.0/MAX_VELOCITY;
	public static final double ACCELERATION_GAIN = 0.0;

    /*-----Elevator Vars-----*/
    public static final double ELEVATOR_ENCODER_DPP = 0.5500928736 * 2.0;
    public static final double ELEVATOR_DOWN_THRESHOLD = 1;
    public static final double ELEVATOR_UP_THRESHOLD = 63;
    public static final double ELEVATOR_SPEED_MULTIPLYER = 1.0;
    public static final double ELEVATOR_UP_KP = 0.1;
    public static final double ELEVATOR_UP_KI = 0;
    public static final double ELEVATOR_UP_KD = 0;
    public static final double ELEVATOR_DOWN_KP = 0.1;
    public static final double ELEVATOR_DOWN_KI = 0;
    public static final double ELEVATOR_DOWN_KD = 0;
    public static final double ELEVATOR_GRAVITY_COMP = 0.5; //volts

    /*-----Hatch Vars-----*/

    /*-----Intake Vars-----*/
    public static final double INTAKE_MOTOR_SPEED = 1.0;

    /*-----Vision Vars-----*/
    public static final int DRIVER_CAM_ID = 0;
    public static final int VISION_CAM_ID = 1;
    public static final double RADIANS_PER_PIXEL = 0.00736;

    /*-----Climber Constants-----*/
    public static final double speed = 0.5;
}