package frc.robot.lib.auto.Pursuiter.helpers;

import java.util.List;


import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;

public class FastMath {
    
    /** Inside float math, be careful of overflow
     * 
     * @param minRequiredDist requires that the minimum distance to a point is beyond this distance in meters.
     * @param curentPose the current robot position
     * @param points to compare
     * @return the closest point without being within the minimum distance.
     */
    public static PathPoint closestPointWithThreshold(float minRequiredDist, Pose2d curentPose, List<PathPoint> points) {
        if (points == null || points.isEmpty()) return null;

        PathPoint closest = points.get(0);
        float minSqrDist = Float.MAX_VALUE;
        float minRequiredSqrDist = minRequiredDist * minRequiredDist;

        float currX = (float) curentPose.getX();
        float currY = (float) curentPose.getY();

        for (int i = 0; i < points.size(); i++) {
            PathPoint p = points.get(i);

            float dx = (float) p.point().getX() - currX;
            float dy = (float) p.point().getY() - currY;
            float squaredDist = dx * dx + dy * dy; 

                if (squaredDist < minSqrDist && squaredDist >= minRequiredSqrDist) {
                    minSqrDist = squaredDist;
                    closest = p;
                }
            }

        return closest;
    }

    public static PathPoint closestPointWithThreshold(float minRequiredDist, PathPoint curentPose, List<PathPoint> points) {
        if (points == null || points.isEmpty()) return null;

        PathPoint closest = points.get(0);
        float minSqrDist = Float.MAX_VALUE;
        float minRequiredSqrDist = minRequiredDist * minRequiredDist;

        float currX = (float) curentPose.point().getX();
        float currY = (float) curentPose.point().getY();

        for (int i = 0; i < points.size(); i++) {
            PathPoint p = points.get(i);

            float dx = (float) p.point().getX() - currX;
            float dy = (float) p.point().getY() - currY;
            float squaredDist = dx * dx + dy * dy; 

                if (squaredDist < minSqrDist && squaredDist >= minRequiredSqrDist) {
                    minSqrDist = squaredDist;
                    closest = p;
                }
            }

        return closest;
    }

    /** Finds the closest PathPoint given a list of PathPoints.
     * 
     * @param curentPose
     * @param points
     */
    public static PathPoint findClosestPoint(Pose2d curentPose, List<PathPoint> points) {
        if (points == null || points.isEmpty()) return null;

        PathPoint closest = null;
        float minSqrDist = Float.MAX_VALUE;

        float currX = (float) curentPose.getX();
        float currY = (float) curentPose.getY();

        for (PathPoint p : points) {
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
