package frc.robot.lib.auto.Pursuiter.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;

public class PursuiterTester {

    public static void allTest() {
        testLoad();
    }
    
    public static void testLoad() {
        List<PathPoint> points = PathLoader.loadSample("L3.traj");
        List<Pose2d> poses = new ArrayList<Pose2d>();

        for (int i = 0; i < points.size(); i += 5) {
            poses.add(points.get(i).point());
            Pose2d[] poseArr = poses.toArray(new Pose2d[0]);
            Logger.recordOutput("Pursuiter/ Test 1: load L3.traj", poseArr);
        }
    }
}
