package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.interfaces.swerve.Swerve;

import org.littletonrobotics.junction.Logger;

public class Limelight {
  private static final String FRONT_LEFT_CAM = "limelight-fl";
  private static final Translation2d FL_OFFSET = new Translation2d(0.0, -0.0);

  private static final String FRONT_RIGHT_CAM = "limelight-fr";
  private static final Translation2d FR_OFFSET = new Translation2d(0.0, 0.0);

  private static final String SIDE_CAM = "limelight-side";
  private static final Translation2d SIDE_OFFSET = new Translation2d(0.0, -0.0);


  public static void init() {
    LimelightHelpers.SetThrottle(FRONT_LEFT_CAM, 0);
    LimelightHelpers.setRewindEnabled(FRONT_LEFT_CAM, false);

    LimelightHelpers.SetThrottle(FRONT_RIGHT_CAM, 0);
    LimelightHelpers.setRewindEnabled(FRONT_RIGHT_CAM, false);

    LimelightHelpers.SetThrottle(SIDE_CAM, 0);
    LimelightHelpers.setRewindEnabled(SIDE_CAM, false);
  }

  public static void periodic() {
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_LEFT_CAM, FL_OFFSET.getX(), FL_OFFSET.getY(), 0, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_RIGHT_CAM, FR_OFFSET.getX(), FR_OFFSET.getY(), 0, 0, 0, 0);
    LimelightHelpers.setCameraPose_RobotSpace(SIDE_CAM, SIDE_OFFSET.getX(), SIDE_OFFSET.getY(), 0, 0, 0, 0);

    if (DriverStation.isDisabled()) {
      seedFromMegaTag1(FRONT_LEFT_CAM);
      seedFromMegaTag1(FRONT_RIGHT_CAM);
      seedFromMegaTag1(SIDE_CAM);
    } else {
      updateWithMegaTag1(FRONT_LEFT_CAM);
      updateWithMegaTag1(FRONT_RIGHT_CAM);
      updateWithMegaTag1(SIDE_CAM);
    }
  }

  private static void seedFromMegaTag1(String cameraName) {
    LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (isValid(mt1Pose)) {
      double xyStdDev = 8; 
      double rotStdDev = 8;
      
      if (mt1Pose.tagCount == 1) {
        xyStdDev *= 3;
        rotStdDev *= 3;
      }

      Swerve.addLimelightMeasurement(
          mt1Pose.pose,
          mt1Pose.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
      );

      Logger.recordOutput("Vision/ Disabled Feed, FROM: " + cameraName, mt1Pose.pose);
    }
  }

  private static void updateWithMegaTag1(String cameraName) {
    LimelightHelpers.PoseEstimate mt1PoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (isValid(mt1PoseEstimate)) {
      double xyStdDev = 8.0 + (Math.pow(mt1PoseEstimate.avgTagDist, 2) * 0.1);

      if (mt1PoseEstimate.tagCount > 1) {
        xyStdDev *= 0.8;
      }

      if (Swerve.getYawRateAsDeg() > 270) {
        xyStdDev *= 2;
      }

      if (mt1PoseEstimate.pose.getX() < 0 || mt1PoseEstimate.pose.getX() > 16.5 || 
          mt1PoseEstimate.pose.getY() < 0 || mt1PoseEstimate.pose.getY() > 8.0) return;

      Swerve.addLimelightMeasurement(
          mt1PoseEstimate.pose,
          mt1PoseEstimate.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
      );

      Logger.recordOutput("Vision/ Enabled Feed, FROM: " + cameraName, mt1PoseEstimate.pose);
    }
  }

  private static boolean isValid(LimelightHelpers.PoseEstimate poseEst) {
    return poseEst != null && poseEst.pose != null && poseEst.tagCount > 0;
  }
}
