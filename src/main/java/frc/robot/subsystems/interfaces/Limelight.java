package frc.robot.subsystems.interfaces;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.lib.util.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.interfaces.swerve.Swerve;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class Limelight {
  private static ArrayList<LimeLightObject> cameras;
  private static LimeLightObject FRONT_CAMERA_MODEL4; // TODO: RENAME CAMERAS;

  public static void init() {
    cameras = new ArrayList<>();

    FRONT_CAMERA_MODEL4 = new LimeLightObject("limelight-front", 0); // TODO: RENAME CAMERAS;
    FRONT_CAMERA_MODEL4.usePigeon = false; // IMPORTANT TO DISABLE THE YAW CORRECTION FROM PIGEON
    
    LimelightHelpers.SetIMUMode(FRONT_CAMERA_MODEL4.cameraName, 3);
    LimelightHelpers.SetIMUAssistAlpha(FRONT_CAMERA_MODEL4.cameraName, 0.001);
    LimelightHelpers.setRewindEnabled(FRONT_CAMERA_MODEL4.cameraName, true);

    cameras.add(FRONT_CAMERA_MODEL4);

    LimelightHelpers.setCameraPose_RobotSpace(FRONT_CAMERA_MODEL4.cameraName,
                                              0.349510, 
                                              -0.048847, 
                                              0, 
                                              0, 
                                              0, 
                                              0);
    }
  
  /**
   * THIS SHOULD NEVER BE USED TO INIT THE PIGEON. THIS IS ONLY FOR PERIODIC UPDATE AND NOT PIGEON FEED.
   */
  public static void periodic() {
    if (DriverStation.isDisabled()) {
      disabledPoseSetup(FRONT_CAMERA_MODEL4, 0);      
      return;
    }
    
    periodicEnabled(FRONT_CAMERA_MODEL4, 0);
  }

  private static void periodicEnabled(LimeLightObject limeLight, double yawOffset) {
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLight.cameraName);
    
    double shortestDistance = Double.POSITIVE_INFINITY;
    int shortest_fidx = -1;

    LimelightResults results = LimelightHelpers.getLatestResults(limeLight.cameraName);
    var num_targets = results.targets_Fiducials.length;

    for (int fidx = 0; fidx < num_targets; fidx++) {
      double tag_distance =
          distanceToTag(toPose3D(results.targets_Fiducials[fidx].targetPose_CameraSpace));
      if (tag_distance < shortestDistance) {
        shortestDistance = tag_distance;
        shortest_fidx = fidx;
      }
    }   

    if (shortest_fidx == -1) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "SHORTEST FIDX UNDETECTED");
      return;
    }

    if (!results.valid) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "INVALID RESULTS");
      return;
    }

    if (num_targets < 1) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "NO TARGETS BUT FIDX DETECTED, NUM TARGETS");
      return;
    }

    if (poseEstimate == null || poseEstimate.pose == null) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ "  + "DISABLED ERROR STATUS", "NULLED POSE MT DISABLED");
      return;
    }

    Pose2d feedPose = new Pose2d(poseEstimate.pose.getTranslation(), new Rotation2d(poseEstimate.pose.getRotation().getRadians() - yawOffset));

    Swerve.addLimelightMeasurement(feedPose, poseEstimate.timestampSeconds, VecBuilder.fill(4, 4, 10000));
  }

  private static void disabledPoseSetup(LimeLightObject limeLight, double yawOffset) {
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLight.cameraName);
    
    double shortestDistance = Double.POSITIVE_INFINITY;
    int shortest_fidx = -1;

    LimelightResults results = LimelightHelpers.getLatestResults(limeLight.cameraName);
    var num_targets = results.targets_Fiducials.length;

    for (int fidx = 0; fidx < num_targets; fidx++) {
      double tag_distance =
          distanceToTag(toPose3D(results.targets_Fiducials[fidx].targetPose_CameraSpace));
      if (tag_distance < shortestDistance) {
        shortestDistance = tag_distance;
        shortest_fidx = fidx;
      }
    }   

    if (shortest_fidx == -1) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "SHORTEST FIDX UNDETECTED");
      return;
    }

    if (!results.valid) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "INVALID RESULTS");
      return;
    }

    if (num_targets < 1) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "NO TARGETS BUT FIDX DETECTED, NUM TARGETS");
      return;
    }

    if (poseEstimate == null || poseEstimate.pose == null) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ "  + "DISABLED ERROR STATUS", "NULLED POSE MT DISABLED");
      return;
    }

    Pose2d feedPose = new Pose2d(poseEstimate.pose.getTranslation(), new Rotation2d(poseEstimate.pose.getRotation().getRadians() - yawOffset));

    Swerve.addLimelightMeasurement(feedPose, poseEstimate.timestampSeconds, VecBuilder.fill(4, 4, 4));
  }

  public static Pose3d toPose3D(double[] inData) {
    if (inData.length < 6) {
      System.err.println("Bad LL 3D Pose Data!");
      return new Pose3d();
    }
    Translation3d tran3d = new Translation3d(inData[0], inData[1], inData[2]);
    Rotation3d r3d =
        new Rotation3d(
            Units.degreesToRadians(inData[3]),
            Units.degreesToRadians(inData[4]),
            Units.degreesToRadians(inData[5]));
    return new Pose3d(tran3d, r3d);
  }

  // finds your distance to AprilTag
  public static double distanceToTag(Pose3d tag_pose) {
    return tag_pose.getTranslation().getDistance(new Translation3d(0, 0, 0));
  }

  public static void LogForPositionTuning(Pose3d limelightPose3d, Pose3d knownPose, String limelight_name, boolean enabled) {
    if (!enabled) {
      return;
    }

    Translation3d limelightTranslation =
        knownPose.getTranslation().minus(limelightPose3d.getTranslation());
    Rotation3d limelightRotation = knownPose.getRotation().minus(limelightPose3d.getRotation());

    Logger.recordOutput("Known Pose", knownPose);
    Logger.recordOutput(limelight_name + "/pose", limelightPose3d);
    Logger.recordOutput(limelight_name + "/XDiff", limelightTranslation.getX());
    Logger.recordOutput(limelight_name + "/YDiff", limelightTranslation.getY());
    Logger.recordOutput(limelight_name + "/ZDiff", limelightTranslation.getZ());

    Logger.recordOutput(
        limelight_name + "/RollDiff",
        Rotation2d.fromRadians(limelightRotation.getX()).getDegrees());

    Logger.recordOutput(
        limelight_name + "/PitchDiff",
        Rotation2d.fromRadians(limelightRotation.getY()).getDegrees());

    Logger.recordOutput(
        limelight_name + "/YawDiff", Rotation2d.fromRadians(limelightRotation.getZ()).getDegrees());
  }

  public static class LimeLightObject {
    final String cameraName;
    final double yawOffset;
    PosewithDeviation results;
    boolean usePigeon = true;
    Pose2d lastValidPose;
    double numValidPoses;

    LimeLightObject(String cameraName, double yawOffset) {
      this.cameraName = cameraName;
      this.yawOffset = yawOffset;
    }
  }

  public static class PosewithDeviation {
    Pose2d latestReadPose;
    double timeStamp;
    double distanceStdDev;
    double angleStdDev;

    PosewithDeviation(Pose2d resultPose, double distanceStdDev, double angleStdDev, double timeStamp) {
      this.latestReadPose = resultPose;
      this.distanceStdDev = distanceStdDev;
      this.angleStdDev = angleStdDev;
      this.timeStamp = timeStamp;
    }
  }
}
