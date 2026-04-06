package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.interfaces.swerve.Swerve;

import org.littletonrobotics.junction.Logger;

public class Limelight {
  // X -> Foward shift, positive means shift return pose backwards
  // Y -> Sidways shift, positive means shift the return pose to the left
  // Z -> Up down shift, positive means shift the return pose towards the ground
  private static final String FRONT_LEFT_CAM = "limelight-fl"; 
  private static final Translation3d FL_OFFSET = new Translation3d(0.3290062, -0.24765, 0.288417);

  private static final String FRONT_RIGHT_CAM = "limelight-fr";
  private static final Translation3d FR_OFFSET = new Translation3d(0.3251962, 0.263525, 0.2633472);

  private static final String BACK_CAM = "limelight-back";
  private static final Translation3d BACK_OFFSET = new Translation3d(0.269, -0.340, 0.362);

  private static final int[] VALID_IDS = {1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 29, 30};

  public static void init() {
    LimelightHelpers.SetThrottle(FRONT_LEFT_CAM, 0);
    LimelightHelpers.SetThrottle(FRONT_RIGHT_CAM, 0);
    LimelightHelpers.SetThrottle(BACK_CAM, 0);

    LimelightHelpers.SetIMUMode(FRONT_LEFT_CAM, 0);
    LimelightHelpers.SetIMUMode(FRONT_RIGHT_CAM, 0);
    LimelightHelpers.SetIMUMode(BACK_CAM, 0);

    LimelightHelpers.setRewindEnabled(FRONT_LEFT_CAM, false);
    LimelightHelpers.setRewindEnabled(FRONT_RIGHT_CAM, false);
    LimelightHelpers.setRewindEnabled(BACK_CAM, false);

    LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_LEFT_CAM, VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_RIGHT_CAM, VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(BACK_CAM, VALID_IDS);
  }

  public static void periodic() {
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_LEFT_CAM, FL_OFFSET.getX(), FL_OFFSET.getY(), FL_OFFSET.getZ(), 0, 20, 0);
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_RIGHT_CAM, FR_OFFSET.getX(), FR_OFFSET.getY(), FR_OFFSET.getZ(), 0, 20, 0);
    LimelightHelpers.setCameraPose_RobotSpace(BACK_CAM, BACK_OFFSET.getX(), BACK_OFFSET.getY(), BACK_OFFSET.getZ(), 0, 0, 90);

    if (DriverStation.isDisabled()) {
      seedFromMegaTag1(FRONT_LEFT_CAM);
      seedFromMegaTag1(FRONT_RIGHT_CAM); 
    } else {
      updateWithMegaTag1(FRONT_LEFT_CAM);
      updateWithMegaTag1(FRONT_RIGHT_CAM);
      updateWithMegaTag1(BACK_CAM);
    }
  }

  private static void seedFromMegaTag1(String cameraName) {
    LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (isValid(mt1Pose)) {
      double xyStdDev = 8; 
      double rotStdDev = 8;
        
        Logger.recordOutput("Vision/ Disabled Feed: " + cameraName, mt1Pose.pose);
        Logger.recordOutput("Vision/ AVG Dist to Tag: " + cameraName, mt1Pose.avgTagDist);
        Logger.recordOutput("Vision/ Latency: " + cameraName, mt1Pose.latency);
        Logger.recordOutput("Vision/ TagCount: " + cameraName, mt1Pose.tagCount);
      
      if (mt1Pose.tagCount < 2) {
        Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "NOT ENOUGH TAGS");
        LED.setSolidRed();
        return; // NO READINGS FOR LESS THAN 2 TAGS
      }

      Swerve.addLimelightMeasurement(
          mt1Pose.pose,
          mt1Pose.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
      );
      LED.setFlashGreen();
    }
  }

  private static void updateWithMegaTag1(String cameraName) {
    LimelightHelpers.SetRobotOrientation(cameraName, Swerve.getYawAsDegrees(), Swerve.getYawRateAsDeg(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt1PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    
      Logger.recordOutput("Vision/ Enabled Feed: " + cameraName, mt1PoseEstimate.pose);
      Logger.recordOutput("Vision/ AVG Dist to Tag: " + cameraName, mt1PoseEstimate.avgTagDist);

    if (isValid(mt1PoseEstimate)) {
      if (mt1PoseEstimate.avgTagDist > 4.5) {
        Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "OUT OF RANGE");
        return;
      }

      double xyStdDev = 4.0 + (Math.pow(mt1PoseEstimate.avgTagDist, 1.0) * 1.0);

      if (mt1PoseEstimate.tagCount > 1) {
        xyStdDev *= 0.8;
      }

      if (Swerve.getYawRateAsDeg() > 270) {
        Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "ROTATION TOO FAST, YAWRATE - " + Swerve.getYawRateAsDeg());
        return;
      }

      if (mt1PoseEstimate.pose.getX() < 0 || mt1PoseEstimate.pose.getX() > 16.5 || 
          mt1PoseEstimate.pose.getY() < 0 || mt1PoseEstimate.pose.getY() > 8.0) {return;}

      Swerve.addLimelightMeasurement(
          mt1PoseEstimate.pose,
          mt1PoseEstimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
      );
    }
  }

  private static boolean isValid(LimelightHelpers.PoseEstimate poseEst) {
    return poseEst != null && poseEst.pose != null && poseEst.tagCount > 0;
  }
}
