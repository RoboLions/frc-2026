package frc.lib.Pursuiter.util;

import edu.wpi.first.math.geometry.Pose2d;

public record PathPoint(
    Pose2d point,
    PointConstraints constraints,
    int pointIndex,
    PursuitEventMarker eventMarker,
    double timeStamp) {

  public PathPoint(
      Pose2d point,
      PointConstraints constraints,
      int pointIndex,
      PursuitEventMarker eventMarker,
      double timeStamp) {
    this.point = point;
    this.constraints = constraints;
    this.pointIndex = pointIndex;
    this.eventMarker = eventMarker;
    this.timeStamp = timeStamp;
  }
}
