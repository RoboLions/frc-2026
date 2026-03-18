package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.subsystems.interfaces.swerve.Swerve;

import org.littletonrobotics.junction.Logger;

public class Limelight {
  private static final String FRONT_CAM = "limelight-front";

  public static void init() {
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_CAM, 0.349510, -0.048847, 0, 0, 0, 0);
    LimelightHelpers.SetIMUAssistAlpha(FRONT_CAM, 0.0005);
    LimelightHelpers.SetIMUMode(FRONT_CAM, 3);
    LimelightHelpers.SetThrottle(FRONT_CAM, 0);
    LimelightHelpers.setRewindEnabled(FRONT_CAM, true);
  }

  public static void periodic() {
    LimelightHelpers.setCameraPose_RobotSpace(FRONT_CAM, 0.349510, -0.048847, 0, 0, 0, 0);

    if (DriverStation.isDisabled()) {
      seedFromMegaTag1(FRONT_CAM);
    } else {
      updateWithMegaTag2(FRONT_CAM, 0.0);
    }
  }

  private static void seedFromMegaTag1(String cameraName) {
    LimelightHelpers.PoseEstimate mt1Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    if (isValid(mt1Pose)) {
      double xyStdDev = 4; 
      double rotStdDev = 5;
      
      if (mt1Pose.tagCount == 1) {
        xyStdDev *= 5;
        rotStdDev *= 5;
      }

      Swerve.addLimelightMeasurement(
          mt1Pose.pose,
          mt1Pose.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev)
      );

      Logger.recordOutput("Vision/ Enabled Feed, FROM: " + cameraName, mt1Pose.pose);
    }
  }

  private static void updateWithMegaTag2(String cameraName, double yawOffset) {
    LimelightHelpers.SetRobotOrientation(cameraName, Swerve.getYawAsDegrees() + yawOffset, 0, 0, 0, 0, 0);

    LimelightHelpers.PoseEstimate mt2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    if (isValid(mt2Pose)) {
      double xyStdDev = 5.0 + (Math.pow(mt2Pose.avgTagDist, 2) * 0.1);

      if (mt2Pose.tagCount > 1) {
        xyStdDev *= 0.5;
      }

      if (Swerve.getYawRateAsDeg() > 720) {
        xyStdDev *= 5;
      }

      Swerve.addLimelightMeasurement(
          mt2Pose.pose,
          mt2Pose.timestampSeconds,
          VecBuilder.fill(xyStdDev, xyStdDev, 9999999)
      );

      Logger.recordOutput("Vision/ Enabled Feed, FROM: " + cameraName, mt2Pose.pose);
    }
  }

  private static boolean isValid(LimelightHelpers.PoseEstimate poseEst) {
    return poseEst != null && poseEst.pose != null && poseEst.tagCount > 0;
  }
}
