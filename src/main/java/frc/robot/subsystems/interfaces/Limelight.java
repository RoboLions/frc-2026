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
  private static final Translation3d FL_OFFSET = new Translation3d(0.0111, -0.133, 0.520);

  private static final String FRONT_RIGHT_CAM = "limelight-fr";
  private static final Translation3d FR_OFFSET = new Translation3d(0.0111, 0.133, 0.520);

  private static final int[] VALID_IDS = {1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 13, 14, 17, 18, 19, 20, 21, 22, 24, 25, 26, 27, 29, 30};

  public static void init() {
    LimelightHelpers.SetThrottle(FRONT_LEFT_CAM, 0);
    LimelightHelpers.SetThrottle(FRONT_RIGHT_CAM, 0);

    LimelightHelpers.SetIMUMode(FRONT_LEFT_CAM, 0);
    LimelightHelpers.SetIMUMode(FRONT_RIGHT_CAM, 0);

    LimelightHelpers.setRewindEnabled(FRONT_LEFT_CAM, false);
    LimelightHelpers.setRewindEnabled(FRONT_RIGHT_CAM, false);

    LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_LEFT_CAM, VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(FRONT_RIGHT_CAM, VALID_IDS);
  }

  public static void periodic
  () {
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_LEFT_CAM, FL_OFFSET.getX(), FL_OFFSET.getY(), FL_OFFSET.getZ(), 0, 15, 180);
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_RIGHT_CAM, FR_OFFSET.getX(), FR_OFFSET.getY(), FR_OFFSET.getZ(), 0, 15, 180);

    if (DriverStation.isDisabled()) {
      seedFromMegaTag1(FRONT_LEFT_CAM);
      seedFromMegaTag1(FRONT_RIGHT_CAM); 
    } else {
      updateWithMegaTag(FRONT_LEFT_CAM);
      updateWithMegaTag(FRONT_RIGHT_CAM);
    }
  }

  private static void seedFromMegaTag1(String cameraName) {
    LimelightHelpers.SetRobotOrientation(cameraName, Swerve.getYawAsDegrees(), Swerve.getYawRateAsDeg(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (isValid(mt1Pose)) {
      double xyStdDev = 8; 
      double rotStdDev = 8;
      
      if (mt1Pose.tagCount < 2) {
        return; // NO READINGS FOR LESS THAN 2 TAGS
      }

      Swerve.addLimelightMeasurement(
          mt1Pose.pose,
          mt1Pose.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
      );

      Logger.recordOutput("Vision/ Disabled Feed: " + cameraName, mt1Pose.pose);
      Logger.recordOutput("Vision/ AVG Dist to Tag: " + cameraName, mt1Pose.avgTagDist);
    }
  }

  private static void updateWithMegaTag(String cameraName) {
    LimelightHelpers.SetRobotOrientation(cameraName, Swerve.getYawAsDegrees(), Swerve.getYawRateAsDeg(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mtPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    
    if (isValid(mtPoseEstimate)) {
      if (mtPoseEstimate.avgTagDist > 4.5) {
        Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "OUT OF RANGE");
        return;
      }

      double xyStdDev = 2.0 + (Math.pow(mtPoseEstimate.avgTagDist, 1.0) * 0.5);

      if (mtPoseEstimate.tagCount > 1) {
        xyStdDev *= 0.3;
      }

      if (Swerve.getYawRateAsDeg() > 270) {
        Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "ROTATION TOO FAST, YAWRATE - " + Swerve.getYawRateAsDeg());
        return;
      }

      if (mtPoseEstimate.pose.getX() < 0 || mtPoseEstimate.pose.getX() > 16.5 || 
          mtPoseEstimate.pose.getY() < 0 || mtPoseEstimate.pose.getY() > 8.0) {return;}

      Swerve.addLimelightMeasurement(
          mtPoseEstimate.pose,
          mtPoseEstimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
      );

      Logger.recordOutput("Vision/ ERROR LOG: " + cameraName, "LOOKS GOOD, POSE ACCEPT");
      Logger.recordOutput("Vision/ Enabled Feed: " + cameraName, mtPoseEstimate.pose);
      Logger.recordOutput("Vision/ AVG Dist to Tag: " + cameraName, mtPoseEstimate.avgTagDist);
    }
  }

  private static boolean isValid(LimelightHelpers.PoseEstimate poseEst) {
    return poseEst != null && poseEst.pose != null && poseEst.tagCount > 0;
  }
}
