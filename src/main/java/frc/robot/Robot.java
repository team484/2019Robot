/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CargoSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.HatchSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.ClimberRailSub;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static final RobotIO robotIO = new RobotIO();

  /*-----Subsystems-----*/
  public static final CargoSub cargoSub = new CargoSub();
  public static final DriveSub driveSub = new DriveSub();
  public static final ElevatorSub elevatorSub = new ElevatorSub();
  public static final HatchSub hatchSub = new HatchSub();
  public static final IntakeSub intakeSub = new IntakeSub();
  public static final ClimberSub climberSub = new ClimberSub();
  public static final ClimberRailSub climberRailSub = new ClimberRailSub();

  /*-----Camera/vision vars-----*/
  private static boolean isCameraServerUp = false;
  private static boolean isDevMode = false;
  public static boolean disableVision = false;
  VisionThread visionThread;
  public static long lastVisionUpdateTime;
  public static double targetAngle = 0.0; // The angle the robot must turn to face the target
  public static double targetDistance = 0.0; // The x coordinate (distance left from middle of robot) of target
  public static boolean targetFound = false; //If robot can see a vision target
  public static final Object imgLock = new Object();

  public static OI oi;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    SmartDashboard.putData(RobotIO.pdp);
    oi = new OI();
    oi.setupOI();
    enableCameraServer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("DriveTrain Left Enc", RobotIO.leftEncoder.getDistance());
    SmartDashboard.putNumber("DriveTrain Right Enc", RobotIO.rightEncoder.getDistance());
    SmartDashboard.putNumber("Elevator Height", ElevatorSub.getHeight());
    SmartDashboard.putNumber("Outtake IR", RobotIO.cargoSensor.getAverageVoltage());
    double[] ypr = new double[3];
    RobotIO.imu.getYawPitchRoll(ypr);
    SmartDashboard.putNumber("Gyro", ypr[0]);

  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    enableCameraServer();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    RobotIO.compressor.stop(); //disable compressor in auto to reduce variability
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    RobotIO.compressor.start();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void enableCameraServer() {
    if (isCameraServerUp)
      return;
    try {
      //-----Driver Camera-----
      UsbCamera driverCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.DRIVER_CAM_ID);
      driverCamera.setExposureAuto();
      driverCamera.setWhiteBalanceAuto();
      driverCamera.setFPS(120);

      //-----Vision Camera-----
      String[] args = new String[] {"/bin/bash", "-c", "v4l2-ctl -d 1 -c exposure_auto=1 -c exposure_absolute=10"};
      Process proc = new ProcessBuilder(args).start();
      proc.getOutputStream();
      UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.VISION_CAM_ID);
      visionCamera.setResolution(RobotSettings.IMG_WIDTH, RobotSettings.IMG_HEIGHT);
      visionCamera.setWhiteBalanceManual(3600);
      //CvSink visionCVSink = CameraServer.getInstance().getVideo(visionCamera);
      //CvSource devStream = CameraServer.getInstance().putVideo("Dev Stream", RobotSettings.IMG_WIDTH, RobotSettings.IMG_HEIGHT);
      
      
      //-----Computer Vision Thread-----
      visionThread = new VisionThread(visionCamera, new TargetVisionPipeline(), pipeline -> {
        if (disableVision || (pipeline.filterContoursOutput().isEmpty() && !isDevMode))
          return;
        ArrayList<VisionTapeResult> lRects = new ArrayList<>(); //used for dev stream
        ArrayList<VisionTapeResult> rRects = new ArrayList<>(); //used for dev stream
        double imgCenterX = RobotSettings.IMG_WIDTH / 2.0;
        
        VisionTapeResult centermostRect = null;
        //-----Loop through contours-----
        SmartDashboard.putNumber("Contours", pipeline.filterContoursOutput().size());
        for (MatOfPoint contour : pipeline.filterContoursOutput()) {
          RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
          double size = rect.size.height * rect.size.width;

          //Left Side Tape
          if (rect.angle > -85 && rect.angle < -65) {
            double width = rect.size.height; //l rects are backwards
            Point[] points = new Point[4];
            rect.points(points);
            Point heighestPt = points[0];
            
            for (Point pt : points) {
              //top of screen is 0 bottom is 480px so lowest y is highest pt
              if (pt.y < heighestPt.y) {
                heighestPt = pt;
              }
            }
            double ppi = width/2.0;
            double targetX = 5.935 * ppi + heighestPt.x;
            double targetY = 12.5 * ppi + heighestPt.y;
            VisionTapeResult result = new VisionTapeResult(rect, targetX, targetY, ppi, heighestPt, true);
            lRects.add(result);
            if (centermostRect == null) { //if this is first rect to be tested
              centermostRect = result;
            } else if (size / 2.0 > centermostRect.rect.size.area()) {//if old rect was too small
              centermostRect = result;
            } else if (Math.abs(centermostRect.targetX - imgCenterX) > Math.abs(targetX - imgCenterX)) { //if this rect has target closer to center
              centermostRect = result;
            }

          //Right Side Tape
          } else if (rect.angle > -25 && rect.angle < -5) {
            double width = rect.size.width; //r rects aren't backwards
            Point[] points = new Point[4];
            rect.points(points);
            Point heighestPt = points[0];
            
            for (Point pt : points) {
              //top of screen is 0 bottom is 480px so lowest y is highest pt
              if (pt.y < heighestPt.y) {
                heighestPt = pt;
              }
            }
            double ppi = width/2.0;
            double targetX = -5.935 * ppi + heighestPt.x;
            double targetY = 12.5 * ppi + heighestPt.y;
            VisionTapeResult result = new VisionTapeResult(rect, targetX, targetY, ppi, heighestPt, false);
            rRects.add(result);
            if (centermostRect == null) { //if this is first rect to be tested
              centermostRect = result;
            } else if (size / 2.0 > centermostRect.rect.size.area()) {//if old rect was too small
              centermostRect = result;
            } else if (Math.abs(centermostRect.targetX - imgCenterX) > Math.abs(targetX - imgCenterX)) { //if this rect has target closer to center
              centermostRect = result;
            }
          }

        }

        VisionTapeResult leftRect = null;
        VisionTapeResult rightRect = null;
        if (centermostRect != null && centermostRect.isLeftRect) {
          leftRect = centermostRect;
          //Find potential right rect to accompany
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
          //Find potential right rect to accompany
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
        double targetX=0;
        double targetY=0;
        double ppi=0;
        boolean targetFound = true;
        double distance = 0;
        double angleX = 0;
        double angleXDeg = 0;
        if (leftRect != null && rightRect != null) {
          //use both tape to find target
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
          //No target found
          targetFound = false;
        }
        if (targetFound) {
          //Calculate target location in real space
          double anglePerInch = ppi * RobotSettings.RADIANS_PER_PIXEL;
          distance = 1.0/Math.tan(anglePerInch) - 15.5;
          angleX = (imgCenterX - targetX) * RobotSettings.RADIANS_PER_PIXEL;
          angleXDeg = Math.toDegrees(angleX);
          Robot.targetAngle = angleXDeg;
          Robot.targetDistance = distance;
        }
        SmartDashboard.putNumber("Vision frame rate (ms)", System.currentTimeMillis() - Robot.lastVisionUpdateTime);
        Robot.targetFound = targetFound;
        Robot.lastVisionUpdateTime = System.currentTimeMillis();
        if (isDevMode) {
          Mat img = new Mat();
          //visionCVSink.grabFrame(img);
          for (VisionTapeResult res : lRects) {
            Point[] rectpts = new Point[4]; res.rect.points(rectpts);
            for (int i = 0; i < 4; i++) {
              Imgproc.line(img, rectpts[i], rectpts[(i+1)%4], new Scalar(255,0,0));
            }
          }
          for (VisionTapeResult res : rRects) {
            Point[] rectpts = new Point[4]; res.rect.points(rectpts);
            for (int i = 0; i < 4; i++) {
              Imgproc.line(img, rectpts[i], rectpts[(i+1)%4], new Scalar(0,0,255));
            }
          }
          if (leftRect != null) {
            Point[] rectpts = new Point[4]; leftRect.rect.points(rectpts);
            for (int i = 0; i < 4; i++) {
              Imgproc.line(img, rectpts[i], rectpts[(i+1)%4], new Scalar(255,100,0));
            }
          }
          if (rightRect != null) {
            Point[] rectpts = new Point[4]; rightRect.rect.points(rectpts);
            for (int i = 0; i < 4; i++) {
              Imgproc.line(img, rectpts[i], rectpts[(i+1)%4], new Scalar(0,100,255));
            }
          }
          if (targetFound) {
            Imgproc.circle(img, new Point(targetX, targetY), (int) (ppi * 8.25), new Scalar(0,255,255));
            Imgproc.line(img, new Point(targetX - 20, targetY), new Point(targetX + 20, targetY), new Scalar(0,2550,255));
            Imgproc.line(img, new Point(targetX, targetY - 20), new Point(targetX, targetY + 20), new Scalar(0,2550,255));
            SmartDashboard.putNumber("distance", distance);
            SmartDashboard.putNumber("angleDeg", angleXDeg);
            SmartDashboard.putBoolean("Target", true);
          } else {
            SmartDashboard.putBoolean("Target", false);
          }
          //devStream.putFrame(img);
        } else {
          SmartDashboard.putNumber("distance", distance);
          SmartDashboard.putNumber("angleDeg", angleXDeg);
          SmartDashboard.putBoolean("Target", targetFound);

        }

      });
      visionThread.start();
      isCameraServerUp = true;
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
