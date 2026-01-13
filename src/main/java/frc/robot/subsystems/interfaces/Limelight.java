package frc.robot.subsystem.interfaces;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.lib.util.LimelightHelpers.LimelightResults;

import java.util.HashMap;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Limelight {

  private static Pose3d known_pose_blue_left =
      new Pose3d(new Translation3d(1.252857, 5.547879, 0.0), new Rotation3d(0, 0, Math.PI));
      
  private static final Map<String, Double> last_timestamps = new HashMap<String, Double>();
  public static boolean enabled = true;

  public static void init() {}

  public static void periodic() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      LimelightHelpers.setCameraPose_RobotSpace("Constants.LIMELIGHT.LEFT", 0.379, 0.097, 0, 0, 0, 0);
    } else { // blue
      
    }

    if (Limelight.enabled) {
      Limelight.updateSwervePoseLimelight("Constants.LIMELIGHT.LEFT", 5);
    }
  }

  public static void updateSwervePoseLimelight(String limelight_name, double yawOffset) {
    double adjustedYaw = 0 - yawOffset; //TODO: Fix this
    double yawRate = 0;

    double base_time = Logger.getTimestamp() / 1000000.0;
    LimelightResults results = LimelightHelpers.getLatestResults(limelight_name);
    var num_targets = results.targets_Fiducials.length;
    double ts = results.timestamp_LIMELIGHT_publish;
    double tl = results.latency_pipeline;
    double tc = results.latency_capture;
    double tj = results.latency_jsonParse;

    Double last_timestamp = last_timestamps.get(limelight_name);
    if (last_timestamp != null && last_timestamp == ts) {
      return;
    }

    last_timestamps.put(limelight_name, ts);

    if (!results.valid) {
      return;
    }

    if (num_targets < 1) {
      return;
    }

    LimelightHelpers.SetRobotOrientation(
        limelight_name, adjustedYaw, yawRate, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTagPose =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight_name);

    Pose2d botPose = megaTagPose.pose;

    /**
     * This not part of LimelightLib! When repasting LimelightLib, do not forget to repaste this
     * specific method! This converts a MegaTag2 array into a Pose3d so that we can use it to derive
     * our pysical limelight Camera Offsets. We will leave the loggers commented as they are only
     * nessecary when finding Camera Offsets.
     */
    Pose3d limelightPose3d = LimelightHelpers.getMT2BotPose3d(limelight_name);

    var known_pose = known_pose_blue_left;

    Translation3d limelightTranslation =
        known_pose.getTranslation().minus(limelightPose3d.getTranslation());
    Rotation3d limelightRotation = known_pose.getRotation().minus(limelightPose3d.getRotation());

    Logger.recordOutput("Known Pose", known_pose);
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

    // Logger.recordOutput(limelight_name + "/botpose", results.getBotPose2d());
    // Logger.recordOutput(
    //     limelight_name + "/botpose_wpiblue", results.getBotPose2d_wpiBlue());
    // Logger.recordOutput(
    // limelight_name + "/botpose_wpire", results.getBotPose2d_wpiRed());

    Logger.recordOutput(limelight_name + "/PoseEstimate", botPose);

    if (botPose.getX() >= 16.54
        || botPose.getX() <= 0.0
        || botPose.getY() >= 8.21
        || botPose.getY() <= 0.0) {
      return;
    }

    double shortestDistance = Double.POSITIVE_INFINITY;
    int shortest_fidx = -1;
    for (int fidx = 0; fidx < num_targets; fidx++) {
      double tag_distance =
          distanceToTag(toPose3D(results.targets_Fiducials[fidx].targetPose_CameraSpace));
      if (tag_distance < shortestDistance) {
        shortestDistance = tag_distance;
        shortest_fidx = fidx;
      }
    }

    if (shortest_fidx == -1) {
      return;
    }

    if (shortestDistance > 5 && DriverStation.isAutonomous()) {
      return;
    }

    if ((shortest_fidx == 1 || shortest_fidx == 2 || shortest_fidx == 11)
        && DriverStation.isAutonomous()) {
      return;
    }

    Logger.recordOutput("Shortest Distance", shortestDistance);

    double angleStdDev = 0.5;

    double distanceStdDev = 0.5 * yawRate + 3.0;

    Logger.recordOutput(limelight_name + "/PoseEstimateFiltered", botPose);

    boolean doRejectUpdate = false;

    if (Math.abs(yawRate) > 720) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      // Swerve.addVisionMeasurement(
      //     megaTagPose.pose,
      //     base_time - (tl / 1000.0) - (tc / 1000.0) - (tj / 1000.0),
      //     VecBuilder.fill(distanceStdDev, distanceStdDev, angleStdDev));
    }
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

  public static double angleToTag(Pose3d tag_pose) {
    double distanceZTargetToCam = tag_pose.getZ();
    double distanceTargetToCam = distanceToTag(tag_pose);
    return 180 / Math.PI * Math.acos(distanceZTargetToCam / distanceTargetToCam);
  }
}
