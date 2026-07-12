package frc.robot.lib.auto.Pursuiter.util;

import edu.wpi.first.math.geometry.Pose2d;

public record PathPoint(Pose2d point, PointConstraints constraints, int pointIndex) {

    public PathPoint(Pose2d point, PointConstraints constraints, int pointIndex) {
        this.point = point;
        this.constraints = constraints;
        this.pointIndex = pointIndex;
    }
}
