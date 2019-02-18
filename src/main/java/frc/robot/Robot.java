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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.JoystickElevator;
import frc.robot.subsystems.CargoSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HatchSub;
import frc.robot.subsystems.IntakeSub;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

public class Robot extends TimedRobot {
  public static final RobotIO robotIO = new RobotIO();

  /*-----Subsystems-----*/
  public static final CargoSub cargoSub = new CargoSub();
  public static final DriveSub driveSub = new DriveSub();
  public static final ElevatorSub elevatorSub = new ElevatorSub();
  public static final HatchSub hatchSub = new HatchSub();
  public static final IntakeSub intakeSub = new IntakeSub();
  public static final ClimberSub climberSub = new ClimberSub();

  /*-----Camera/vision vars-----*/
  private static boolean isCameraServerUp = false;
  public static boolean disableVision = false;
  VisionThread visionThread;
  public static long lastVisionUpdateTime;
  public static double targetAngle = 0.0; // The angle the robot must turn to face the target
  public static double targetDistance = 0.0; // The x coordinate (distance left from middle of robot) of target
  public static boolean targetFound = false; // If robot can see a vision target

  public static OI oi;
  private static Command autonCommand;
	private static SendableChooser<Integer> position = new SendableChooser<>();
	private static SendableChooser<Command> auton = new SendableChooser<>();

  @Override
  public void robotInit() {
    SmartDashboard.putData(RobotIO.pdp);
    oi = new OI();
    oi.setupOI();
    enableCameraServer();
    position.addOption("Left L2 (1)", 1);
    position.addOption("Left (2)", 2);
    position.addOption("Center (3)", 3);
    position.addOption("Right L2 (4)", 4);
    position.addOption("Right (5)", 5);
    updateChooser();
    GenerateTrajectory.forceRegen = true;
    GenerateTrajectory.execute("L2ToShipFront",
      new Waypoint(119, 18.75, Pathfinder.d2r(90)),
      new Waypoint(150.125, 190, Pathfinder.d2r(90)));
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Elevator Height", ElevatorSub.getHeight());
    double[] ypr = new double[3];
    SmartDashboard.putNumber("Gyro", ypr[0]);
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
    autonCommand = auton.getSelected();
    if (autonCommand == null) {
      return;
    }
    RobotIO.compressor.stop(); // disable compressor in auto to reduce variability
    autonCommand.start();
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    if (autonCommand != null && autonCommand.isCompleted()) {
      RobotIO.compressor.start();
    }
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
    RobotIO.compressor.start();
    Command elevatorCmd = new JoystickElevator();
    elevatorCmd.start();
    elevatorCmd.close();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
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
        break;
      case 2: //Left
        break;
      case 3: //Center
        break;
      case 4: //Right L2
        break;
      case 5: //Right
        break;
      default:
      break;
    }
    SmartDashboard.putData("Auton", auton);
  }
  private void enableCameraServer() {
    if (isCameraServerUp)
      return;
    try {
      // -----Driver Camera-----
      UsbCamera driverCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.DRIVER_CAM_ID);
      driverCamera.setExposureAuto();
      driverCamera.setWhiteBalanceAuto();
      driverCamera.setFPS(120);

      // -----Vision Camera-----
      String[] args = new String[] { "/bin/bash", "-c", "v4l2-ctl -d 1 -c exposure_auto=1 -c exposure_absolute=10" };
      Process proc = new ProcessBuilder(args).start();
      proc.getOutputStream();
      UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.VISION_CAM_ID);
      visionCamera.setResolution(RobotSettings.IMG_WIDTH, RobotSettings.IMG_HEIGHT);
      visionCamera.setWhiteBalanceManual(3600);

      // -----Computer Vision Thread-----
      visionThread = new VisionThread(visionCamera, new TargetVisionPipeline(), pipeline -> {
        if (disableVision || (pipeline.filterContoursOutput().isEmpty()))
          return;
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
        }
        SmartDashboard.putNumber("Vision frame rate (ms)", System.currentTimeMillis() - Robot.lastVisionUpdateTime);
        Robot.targetFound = targetFound;
        Robot.lastVisionUpdateTime = System.currentTimeMillis();
        SmartDashboard.putNumber("distance", distance);
        SmartDashboard.putNumber("angleDeg", angleXDeg);
        SmartDashboard.putBoolean("Target", targetFound);
      });
      visionThread.start();
      isCameraServerUp = true;
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
