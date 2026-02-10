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
import frc.robot.subsystems.swerve.Swerve;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class Limelight {
  private static ArrayList<LimeLightObject> cameras;
  private static LimeLightObject FRONT_CAMERA_MODEL4; // TODO: RENAME CAMERAS;
  private static LimeLightObject SIDE_CAMERA_MODEL3G; // TODO: RENAME CAMERAS

  public static void init() {
    cameras = new ArrayList<>();

    FRONT_CAMERA_MODEL4 = new LimeLightObject("limelight-four", 0); // TODO: RENAME CAMERAS;
    SIDE_CAMERA_MODEL3G = new LimeLightObject("LL_LEFT", 0); // TODO: RENAME CAMERAS

    FRONT_CAMERA_MODEL4.usePigeon = false; // IMPORTANT TO DISABLE THE YAW CORRECTION FROM PIGEON
    LimelightHelpers.SetIMUMode(FRONT_CAMERA_MODEL4.cameraName, 3);
    LimelightHelpers.SetIMUAssistAlpha(FRONT_CAMERA_MODEL4.cameraName, 0.001);

    cameras.add(FRONT_CAMERA_MODEL4);
    cameras.add(SIDE_CAMERA_MODEL3G);

    LimelightHelpers.setCameraPose_RobotSpace(FRONT_CAMERA_MODEL4.cameraName,
                                              0.356, 
                                              -0.13, 
                                              0, 
                                              0, 
                                              0, 
                                              0);
    LimelightHelpers.setCameraPose_RobotSpace(SIDE_CAMERA_MODEL3G.cameraName,
                                              0.0, 
                                              0.0, 
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
      Logger.recordOutput("LIMELIGHTS /IS ENABLED?", false);
      
      return;
    } else {
      Logger.recordOutput("LIMELIGHTS /IS ENABLED?", true);
    }

    for (LimeLightObject camera : cameras) {

      camera.results = updateCameraResults(camera, camera.yawOffset, Swerve.getYawAsDegrees(), Swerve.getYawRateAsDeg());

      if (camera.results == null) {
        continue;
      }

      double distStdDev = camera.results.distanceStdDev;
      double angleStdDev = camera.results.angleStdDev;
      
      Swerve.addLimelightMeasurement(camera.results.latestReadPose, 
                                     camera.results.timeStamp, 
                                     VecBuilder.fill(distStdDev, distStdDev, angleStdDev));
    }
  }

  private static PosewithDeviation updateCameraResults(LimeLightObject limeLight, double yawOffset, double yawDeg, double yawRate) {
    String limelight_name = limeLight.cameraName;
    
    if (limeLight.usePigeon) {
      double adjustedYaw = yawDeg - yawOffset;

      LimelightHelpers.SetRobotOrientation(
        limelight_name, adjustedYaw, 0, 0, 0, 0, 0);
    }

    double shortestDistance = Double.POSITIVE_INFINITY;
    int shortest_fidx = -1;

    LimelightResults results = LimelightHelpers.getLatestResults(limelight_name);
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
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "SHORTEST FIDX UNDETECTED");
      return null;
    }

    if (shortestDistance > 6) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "SHORTEST DISTANCE THRESHOLD: " + shortestDistance);
      return null;
    }

    if (!results.valid) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "INVALID RESULTS");
      return null;
    }

    if (num_targets < 1) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "NO TARGETS BUT FIDX DETECTED, NUM TARGETS");
      return null;
    }
    
    if (Math.abs(yawRate) > 720) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "ROTATION TOO FAST: " + yawRate);
      return null;
    }

    LimelightHelpers.PoseEstimate megaTagPoseEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name);
    Pose2d botPose = megaTagPoseEstimate.pose;

    
    if (megaTagPoseEstimate.tagCount < 1) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "NO TARGETS BUT FIDX DETECTED, FROM MT2");
      return null;
    }

    if (megaTagPoseEstimate.pose == null) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "NULLED POSE MT2");
      return null;
    }

    if (Math.abs(megaTagPoseEstimate.pose.getRotation().getDegrees() - yawDeg) > 10) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "YAW DEVIATION TOO LARGE"  );
      return null;
    }


    if (botPose.getX() >= 17 // TODO: GET A NEW FIELD RANGE
        || botPose.getY() >= 9
        || botPose.getX() <= -0.5
        || botPose.getY() <= -0.5) {
      Logger.recordOutput(limelight_name + "RETURNS/ "  + " ERROR STATUS", "POSE OUT OF FIELD");
      return null;
    }

    Logger.recordOutput(limelight_name + "RETURNS/ "  + "/Last Raw-Pose", botPose);
    Logger.recordOutput(limelight_name + "RETURNS/ "  + "/Shortest Distance", shortestDistance);

    /**
     * This not part of LimelightLib! When repasting LimelightLib, do not forget to repaste this
     * specific method! This converts a MegaTag2 array into a Pose3d so that we can use it to derive
     * our pysical limelight Camera Offsets. We will leave the loggers commented as they are only
     * nessecary when finding Camera Offsets.
     */
    Pose3d limelightPose3d = LimelightHelpers.getMT2BotPose3d(limelight_name);

    LogForPositionTuning(limelightPose3d, Constants.LimeLight.known_pose_blue_left, limelight_name, false);

    double angleStdDev = 1000000;
    double distanceStdDev = 0.25 * Math.abs(yawRate) + 5.0;

    Logger.recordOutput(limelight_name + "RETURNS/ "  + "/Distance Deviation", distanceStdDev);
    Logger.recordOutput(limelight_name + "RETURNS/ "  + "/Angle Deviation", angleStdDev);
    
    return new PosewithDeviation(botPose, 
                                 distanceStdDev, 
                                 angleStdDev, 
                                 megaTagPoseEstimate.timestampSeconds);
  }

  private static void disabledPoseSetup(LimeLightObject limeLight, double yawOffset) {
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLight.cameraName);
    
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

    if (shortestDistance > 6) {
      Logger.recordOutput(limeLight.cameraName + "RETURNS/ " + "DISABLED ERROR STATUS", "SHORTEST DISTANCE THRESHOLD: " + shortestDistance);
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

    Swerve.addLimelightMeasurement(feedPose, poseEstimate.timestampSeconds, VecBuilder.fill(7, 7, 4));
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
