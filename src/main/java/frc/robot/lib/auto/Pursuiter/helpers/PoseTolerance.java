package frc.robot.lib.auto.Pursuiter.helpers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public record PoseTolerance(Distance metersTolerance, Angle degTolerance) { // lol thanks 1678

  public PoseTolerance(Distance metersTolerance, Angle degTolerance) {
    this.metersTolerance = metersTolerance;
    this.degTolerance = degTolerance;
  }

  public boolean inError(Pose2d targetPose, Pose2d currentPose) {
    double currError = currentPose.getTranslation().getDistance(targetPose.getTranslation());

    return MathUtil.isNear(0, currError, metersTolerance.in(Meters))
        && MathUtil.isNear(
            targetPose.getRotation().getDegrees(),
            currentPose.getRotation().getDegrees(),
            degTolerance.in(Degrees),
            -180,
            180);
  }
}
