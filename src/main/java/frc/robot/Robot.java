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
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
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

  /*-----Camera/vision vars-----*/
  private static boolean isCameraServerUp = false;
  private static boolean isDevMode = true;
  public static boolean disableVision = false;
  VisionThread visionThread;
  public static double targetAngle = 0.0; // The angle the robot must turn to face the target
  public static double targetX = 0.0; // The x coordinate (distance left from middle of robot) of target
  public static double targetY = 0.0; // The y coordinate (distance forward from front of robot) of target
  public static final Object imgLock = new Object();
  public static long lastVisionUpdate = 0;

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
    visionThread.start();
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
      UsbCamera visionCamera = CameraServer.getInstance().startAutomaticCapture(RobotSettings.VISION_CAM_ID);
      visionCamera.setResolution(640, 480);
      CvSource devStream = CameraServer.getInstance().putVideo("Dev Stream", 640, 480);

      //-----Computer Vision Thread-----
      visionThread = new VisionThread(visionCamera, new TargetVisionPipeline(), pipeline -> {
        if (disableVision || pipeline.filterContoursOutput().isEmpty())
          return;

        ArrayList<MatOfPoint> lRects = new ArrayList<>(); //used for dev stream
        ArrayList<MatOfPoint> rRects = new ArrayList<>(); //used for dev stream

        RotatedRect largestLRect = null;
        RotatedRect largestRRect = null;
        double largestLRectSize = -1;
        double largestRRectSize = -1;
        //-----Loop through contours-----
        for (MatOfPoint contour : pipeline.filterContoursOutput()) {
          RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
          double size = rect.size.height * rect.size.width;

          if (rect.angle > -25 && rect.angle < -5) {
            if (size > largestLRectSize) {
              largestLRect = rect;
              largestLRectSize = size;
              if (isDevMode) {
                MatOfPoint boxPts = new MatOfPoint();
                Imgproc.boxPoints(rect, boxPts);
                lRects.add(boxPts);
              }
            }

          } else if (rect.angle > -85 && rect.angle < -65) {
            if (size > largestRRectSize) {
              largestRRect = rect;
              largestRRectSize = size;
              if (isDevMode) {
                MatOfPoint boxPts = new MatOfPoint();
                Imgproc.boxPoints(rect, boxPts);
                rRects.add(boxPts);
              }
            }
          }

        }

        if (largestLRectSize > largestRRectSize) {
          if (largestRRect != null && largestLRect.center.x < largestRRect.center.x && largestLRectSize < largestRRectSize * 2.5) {
            //We are good to use both rects to determine distance
            double lRectAngleX = RobotSettings.RADIANS_PER_PIXEL * (320 - largestLRect.center.x);
            double lRectAngleY = RobotSettings.RADIANS_PER_PIXEL * (240 - largestLRect.center.y);
            double rRectAngleX = RobotSettings.RADIANS_PER_PIXEL * (320 - largestRRect.center.x);
            double rRectAngleY = RobotSettings.RADIANS_PER_PIXEL * (240 - largestRRect.center.y);
            double distance = 5.5 / Math.tan((lRectAngleX - rRectAngleX) / 2.0);
            SmartDashboard.putNumber("TargetDistance", distance);
            SmartDashboard.putNumber("TargetAngle", (lRectAngleX + rRectAngleX)/2.0); //Not actual angle but close
            
          }
        }

        if (largestRRectSize > largestLRectSize) {
          if (largestLRect != null && largestLRect.center.x < largestRRect.center.x && largestRRectSize < largestLRectSize * 2.5) {
            //We are good to use both rects to determine distance
            double lRectAngleX = RobotSettings.RADIANS_PER_PIXEL * (320 - largestLRect.center.x);
            double lRectAngleY = RobotSettings.RADIANS_PER_PIXEL * (240 - largestLRect.center.y);
            double rRectAngleX = RobotSettings.RADIANS_PER_PIXEL * (320 - largestRRect.center.x);
            double rRectAngleY = RobotSettings.RADIANS_PER_PIXEL * (240 - largestRRect.center.y);
            double distance = 5.5 / Math.tan((lRectAngleX - rRectAngleX) / 2.0);
            SmartDashboard.putNumber("TargetDistance", distance);
            SmartDashboard.putNumber("TargetAngle", (lRectAngleX + rRectAngleX)/2.0); //Not actual angle but close
            
          }
        }
        
        if (isDevMode) {
          Mat img = pipeline.startImage();
          Imgproc.drawContours(img, lRects, 0, new Scalar(255, 0, 0));
          Imgproc.drawContours(img, rRects, 0, new Scalar(0, 0, 255));
          devStream.putFrame(img);
          if (largestLRect != null) {
            SmartDashboard.putNumber("LRectSize", largestLRectSize);
            SmartDashboard.putNumber("LRectX", largestLRect.center.x);
            SmartDashboard.putNumber("LRectY", largestLRect.center.y);
            SmartDashboard.putNumber("LRectWidth", largestLRect.size.width);
            SmartDashboard.putNumber("LRectHeight", largestLRect.size.height);
            SmartDashboard.putNumber("LRectAngle", largestLRect.angle);
          }
          if (largestRRect != null) {
            SmartDashboard.putNumber("RRectSize", largestRRectSize);
            SmartDashboard.putNumber("RRectX", largestRRect.center.x);
            SmartDashboard.putNumber("RRectY", largestRRect.center.y);
            SmartDashboard.putNumber("RRectWidth", largestRRect.size.width);
            SmartDashboard.putNumber("RRectHeight", largestRRect.size.height);
            SmartDashboard.putNumber("RRectAngle", largestRRect.angle);
          }
        }

      });

      isCameraServerUp = true;
    } catch (Exception e) {
      e.printStackTrace();
    }
  }
}
