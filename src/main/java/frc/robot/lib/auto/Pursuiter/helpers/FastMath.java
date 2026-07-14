package frc.robot.lib.auto.Pursuiter.helpers;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;
import java.util.List;

public class FastMath {

  /**
   * Inside float math, be careful of overflow
   *
   * @param minRequiredDist requires that the minimum distance to a point is beyond this distance in
   *     meters.
   * @param curentPose the current robot position
   * @param points to compare
   * @return the closest point without being within the minimum distance.
   */
  public static PathPoint closestPointWithThreshold(
      float minRequiredDist, PathPoint currPoint, List<PathPoint> points) {
    if (points == null || points.isEmpty()) return null;

    PathPoint closest = points.get(points.size() - 1);
    float minSqrDist = Float.MAX_VALUE;
    float minRequiredSqrDist = minRequiredDist * minRequiredDist;

    float currX = (float) currPoint.point().getX();
    float currY = (float) currPoint.point().getY();

    for (int i = 0; i < points.size(); i++) {
      PathPoint p = points.get(i);

      float dx = (float) p.point().getX() - currX;
      float dy = (float) p.point().getY() - currY;
      float squaredDist = dx * dx + dy * dy;

      if (squaredDist < minSqrDist && squaredDist >= minRequiredSqrDist) {
        minSqrDist = squaredDist;
        closest = p;
        return closest;
      }
    }

    return closest;
  }

  /**
   * Finds the closest PathPoint given a list of PathPoints.
   *
   * @param curentPose
   * @param points
   */
  public static PathPoint findClosestPoint(
      Pose2d curentPose, List<PathPoint> points, int prevCurr, int prevLookAheadIndex) {
    if (points == null || points.isEmpty()) return null;

    PathPoint closest = null;
    float minSqrDist = Float.MAX_VALUE;

    float currX = (float) curentPose.getX();
    float currY = (float) curentPose.getY();

    List<PathPoint> searchablePts = points.subList(prevCurr, prevLookAheadIndex + 1);

    for (PathPoint p : searchablePts) {
      float dx = (float) p.point().getX() - currX;
      float dy = (float) p.point().getY() - currY;
      float squaredDist = dx * dx + dy * dy;

      if (squaredDist < minSqrDist) {
        minSqrDist = squaredDist;
        closest = p;
      }
    }

    return closest;
  }
}
