/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.JoystickElevator;
import frc.robot.commands.auto.L1CargoshipFront;
import frc.robot.commands.auto.L2CargoshipFront;
import frc.robot.commands.auto.L2CargoshipFrontHold;
import frc.robot.commands.auto.L2CargoshipSide;
import frc.robot.commands.auto.L2CargoshipSideSpline;
import frc.robot.subsystems.CargoSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HatchSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LEDSub;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class Robot extends TimedRobot {

  public Robot() {
    super(0.05);
  }
  public static final RobotIO robotIO = new RobotIO();

  /*-----Subsystems-----*/
  public static final CargoSub cargoSub = new CargoSub();
  public static final DriveSub driveSub = new DriveSub();
  public static final ElevatorSub elevatorSub = new ElevatorSub();
  public static final HatchSub hatchSub = new HatchSub();
  public static final IntakeSub intakeSub = new IntakeSub();
  public static final ClimberSub climberSub = new ClimberSub();
  public static final LEDSub ledSub = new LEDSub();

  /*-----Camera/vision vars-----*/
  private static boolean isCameraServerUp = false;
  public static boolean disableVision = true;
  public static boolean visionUpToDate = false;
  VisionThread visionThread;
  public static long lastVisionUpdateTime;
  public static double targetAngle = 0.0; // The angle the robot must turn to face the target
  public static double targetGyroAngle = 0;
  public static double targetDistance = 0.0; // The x coordinate (distance left from middle of robot) of target
  public static boolean targetFound = false; // If robot can see a vision target

  public static long matchStartTime = 0;

  public static OI oi;
  private static Command autonCommand;
	private static SendableChooser<Integer> position = new SendableChooser<>();
	private static SendableChooser<Command> auton = new SendableChooser<>();

  @Override
  public void robotInit() {
    matchStartTime = System.currentTimeMillis();
    oi = new OI();
    oi.setupOI();
    enableCameraServer();
    position.addOption("Left L2 (1)", 1);
    position.addOption("Left (2)", 2);
    position.addOption("Center (3)", 3);
    position.addOption("Right L2 (4)", 4);
    position.addOption("Right (5)", 5);
    SmartDashboard.putData("Position", position);
    updateChooser();
    GenerateTrajectory.forceRegen = false;
    // Do 322 - x_coord to get mirror image
    GenerateTrajectory.execute("testPath",
      new Waypoint(0, 0, Pathfinder.d2r(90)),
      new Waypoint(0, 40, Pathfinder.d2r(90)));
      
    GenerateTrajectory.execute("L2CargoShipSideLeft",
      new Waypoint(119, 0.75, Pathfinder.d2r(90)),
      new Waypoint(80, 215, Pathfinder.d2r(90)),
      new Waypoint(113, 283, Pathfinder.d2r(5)));

    GenerateTrajectory.execute("L2CargoShipSideRight",
      new Waypoint(322-119, 0.75, Pathfinder.d2r(90)),
      new Waypoint(322-80, 215, Pathfinder.d2r(90)),
      new Waypoint(322-113, 283, Pathfinder.d2r(5)));
      
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Elevator Height", ElevatorSub.getHeight());
    double[] ypr = new double[3];
    SmartDashboard.putNumber("Gyro", ypr[0]);
    SmartDashboard.putNumber("Pitch", ypr[1]);
    SmartDashboard.putNumber("Roll", ypr[2]);

    updateChooser();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    enableCameraServer();
  }

  @Override
  public void autonomousInit() {
    RobotIO.imu.setYaw(0);
    matchStartTime = System.currentTimeMillis();
    autonCommand = auton.getSelected();
    if (autonCommand == null) {
      return;
    }
    autonCommand.start();
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    if (Math.abs(RobotIO.driverStick.getMagnitude()) > 0.5
      || Math.abs(RobotIO.hatchStick.getMagnitude()) > 0.5
      || Math.abs(RobotIO.cargoStick.getMagnitude()) > 0.5) {
        if (autonCommand != null && autonCommand.isRunning()) {
          autonCommand.cancel();
        }
      }
  }

  @Override
  public void teleopInit() {
    if (autonCommand != null && autonCommand.isRunning()) {
      autonCommand.cancel();
    }
    Command elevatorCmd = new JoystickElevator();
    elevatorCmd.start();
    elevatorCmd.close();
    DriveSub.setVoltageCompensation(false);
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    if (RobotIO.driverStick.getRawButton(11)) {
      matchStartTime = 0;
    }
  }

  @Override
  public void testPeriodic() {
  }

  private static int oldPos = -1;
  private static void updateChooser() {
    if (position.getSelected() == null) {
      return;
    }
    int newPos = position.getSelected();
    if (oldPos == newPos) {
      return;
    }
    oldPos = newPos;
    SmartDashboard.delete("Auton");
    auton = new SendableChooser<Command>();
    auton.addOption("Do Nothing", null);
    switch(newPos) {
      case 1: //Left L2
      auton.addOption("Cargoship Front", new L2CargoshipFront(true));
      auton.addOption("Cargoship Front Hold", new L2CargoshipFrontHold(true));
      auton.addOption("Cargoship Side", new L2CargoshipSide(true));
      auton.addOption("Cargoship Side Cargo", new L2CargoshipSideSpline(true));
        break;
      case 2: //Left
        auton.addOption("Cargoship Front", new L1CargoshipFront(true));
        break;
      case 3: //Center
        break;
      case 4: //Right L2
        auton.addOption("Cargoship Front", new L2CargoshipFront(false));
        auton.addOption("Cargoship Front Hold", new L2CargoshipFrontHold(false));
        auton.addOption("Cargoship Side", new L2CargoshipSide(false));
        auton.addOption("Cargoship Side Cargo", new L2CargoshipSideSpline(false));

        break;
      case 5: //Right
        auton.addOption("Cargoship Front", new L1CargoshipFront(false));
        break;
      default:
      break;
    }
    SmartDashboard.putData("Auton", auton);
  }

  public static double lastGyroHeading = 0;
  private void enableCameraServer() {
    if (isCameraServerUp)
      return;
    try {
      // -----Driver Camera-----
      UsbCamera driverCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.DRIVER_CAM_ID);
      driverCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      driverCamera.setPixelFormat(PixelFormat.kMJPEG);
      driverCamera.setFPS(120);
      driverCamera.setResolution(320, 240);

      // -----Vision Camera-----
      //String[] args = new String[] { "/bin/bash", "-c", "v4l2-ctl -d 1 -c exposure_auto=1 -c exposure_absolute=10" };
      //Process proc = new ProcessBuilder(args).start();
      //proc.getOutputStream();
      UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.VISION_CAM_ID);
      visionCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      visionCamera.setResolution(RobotSettings.IMG_WIDTH, RobotSettings.IMG_HEIGHT);
      visionCamera.setWhiteBalanceManual(3600);

      // -----Computer Vision Thread-----
      visionThread = new VisionThread(visionCamera, new TargetVisionPipeline(), pipeline -> {
        if (Robot.disableVision || (pipeline.filterContoursOutput().isEmpty())) {
          targetFound = false;
          targetDistance = 0;
          targetAngle = 0;
          Robot.visionUpToDate = !Robot.disableVision;
          lastGyroHeading = DriveSub.getHeading();
          return;
        }
        ArrayList<VisionTapeResult> lRects = new ArrayList<>(); // used for dev stream
        ArrayList<VisionTapeResult> rRects = new ArrayList<>(); // used for dev stream
        double imgCenterX = RobotSettings.IMG_WIDTH / 2.0;

        VisionTapeResult centermostRect = null;
        // -----Loop through contours-----
        SmartDashboard.putNumber("Contours", pipeline.filterContoursOutput().size());
        for (MatOfPoint contour : pipeline.filterContoursOutput()) {
          RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
          double size = rect.size.height * rect.size.width;

          // Left Side Tape
          if (rect.angle > -85 && rect.angle < -65) {
            double width = rect.size.height; // l rects are backwards
            Point[] points = new Point[4];
            rect.points(points);
            Point heighestPt = points[0];

            for (Point pt : points) {
              // top of screen is 0 bottom is 480px so lowest y is highest pt
              if (pt.y < heighestPt.y) {
                heighestPt = pt;
              }
            }
            double ppi = width / 2.0;
            double targetX = 5.935 * ppi + heighestPt.x;
            double targetY = 12.5 * ppi + heighestPt.y;
            VisionTapeResult result = new VisionTapeResult(rect, targetX, targetY, ppi, heighestPt, true);
            lRects.add(result);
            if (centermostRect == null) { // if this is first rect to be tested
              centermostRect = result;
            } else if (size / 2.0 > centermostRect.rect.size.area()) {// if old rect was too small
              centermostRect = result;
            } else if (Math.abs(centermostRect.targetX - imgCenterX) > Math.abs(targetX - imgCenterX)) {
              centermostRect = result;
            }

            // Right Side Tape
          } else if (rect.angle > -25 && rect.angle < -5) {
            double width = rect.size.width; // r rects aren't backwards
            Point[] points = new Point[4];
            rect.points(points);
            Point heighestPt = points[0];

            for (Point pt : points) {
              // top of screen is 0 bottom is 480px so lowest y is highest pt
              if (pt.y < heighestPt.y) {
                heighestPt = pt;
              }
            }
            double ppi = width / 2.0;
            double targetX = -5.935 * ppi + heighestPt.x;
            double targetY = 12.5 * ppi + heighestPt.y;
            VisionTapeResult result = new VisionTapeResult(rect, targetX, targetY, ppi, heighestPt, false);
            rRects.add(result);
            if (centermostRect == null) { // if this is first rect to be tested
              centermostRect = result;
            } else if (size / 2.0 > centermostRect.rect.size.area()) {// if old rect was too small
              centermostRect = result;
            } else if (Math.abs(centermostRect.targetX - imgCenterX) > Math.abs(targetX - imgCenterX)) {
              centermostRect = result;
            }
          }

        }

        VisionTapeResult leftRect = null;
        VisionTapeResult rightRect = null;
        if (centermostRect != null && centermostRect.isLeftRect) {
          leftRect = centermostRect;
          // Find potential right rect to accompany
          VisionTapeResult nearestRect = null;
          for (VisionTapeResult rect : rRects) {
            if (nearestRect == null) {
              nearestRect = rect;
            } else if (centermostRect.getDistance(nearestRect) > centermostRect.getDistance(rect)) {
              nearestRect = rect;
            }
          }
          if (nearestRect != null && centermostRect.getDistance(nearestRect) < 4.0) {
            rightRect = nearestRect;
          }
        } else if (centermostRect != null) {
          rightRect = centermostRect;
          // Find potential right rect to accompany
          VisionTapeResult nearestRect = null;
          for (VisionTapeResult rect : lRects) {
            if (nearestRect == null) {
              nearestRect = rect;
            } else if (centermostRect.getDistance(nearestRect) > centermostRect.getDistance(rect)) {
              nearestRect = rect;
            }
          }
          if (nearestRect != null && centermostRect.getDistance(nearestRect) < 4.0) {
            leftRect = nearestRect;
          }
        }
        double targetX = 0;
        double targetY = 0;
        double ppi = 0;
        boolean targetFound = true;
        double distance = 0;
        double angleX = 0;
        double angleXDeg = 0;
        if (leftRect != null && rightRect != null) {
          // use both tape to find target
          targetX = (leftRect.outsideEdge.x + rightRect.outsideEdge.x) / 2.0;
          ppi = (rightRect.outsideEdge.x - leftRect.outsideEdge.x) / 11.87;
          targetY = (leftRect.outsideEdge.y + rightRect.outsideEdge.y) / 2.0 + 12.5 * ppi;
        } else if (leftRect != null) {
          targetX = leftRect.targetX;
          targetY = leftRect.targetY;
          ppi = leftRect.ppi;
        } else if (rightRect != null) {
          targetX = rightRect.targetX;
          targetY = rightRect.targetY;
          ppi = rightRect.ppi;
        } else {
          // No target found
          targetFound = false;
        }
        if (targetFound) {
          // Calculate target location in real space
          double anglePerInch = ppi * RobotSettings.RADIANS_PER_PIXEL;
          distance = 1.0 / Math.tan(anglePerInch) - 15.5;
          angleX = -(imgCenterX - targetX) * RobotSettings.RADIANS_PER_PIXEL;
          angleXDeg = Math.toDegrees(angleX);
          Robot.targetAngle = angleXDeg - 1.3;
          Robot.targetDistance = distance;
        } else {
          Robot.targetDistance = 0;
          Robot.targetAngle = 0;
        }
        Robot.targetGyroAngle = targetAngle + lastGyroHeading;
        lastGyroHeading = DriveSub.getHeading();
        Robot.targetFound = targetFound;
        Robot.visionUpToDate = true;
        SmartDashboard.putNumber("Vision frame rate (ms)", System.currentTimeMillis() - Robot.lastVisionUpdateTime);
        Robot.lastVisionUpdateTime = System.currentTimeMillis();
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angleDeg", angleXDeg);
        SmartDashboard.putBoolean("Target", targetFound);
      });
      visionThread.start();
      isCameraServerUp = true;
    } catch (Exception e) {
      ErrorManager.add("Vision Error " + e.getMessage());
      e.printStackTrace();
    }
  }
}
