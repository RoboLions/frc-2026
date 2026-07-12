package frc.robot.lib.auto.Pursuiter.util;

import edu.wpi.first.math.geometry.Pose2d;

public record PathPoint(Pose2d point, PointConstraints constraints) {

    public PathPoint(Pose2d point, PointConstraints constraints) {
        this.point = point;
        this.constraints = constraints;
    }
}
