package frc.robot.lib.auto.Pursuiter.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.helpers.FastMath;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class PursuiterTester {

    static PursuitPath L3 = new PursuitPath(
            0.5, 
            10, 
            Units.Meters.of(0.3), 
            new PIDController(0.5, 0, 0), 
            new PIDController(0.5, 0, 0), 
            new PIDController(3.0, 0, 0), 
            "L3.traj");

    public static void initTest() {
        testLoadInit();
        Swerve.resetPose(L3.getCurrentPoint().point());
    }
    
    public static void testLoadInit() {
        List<PathPoint> points = PathLoader.loadSample("L3.traj");
        List<Pose2d> poses = new ArrayList<Pose2d>();

        for (int i = 0; i < points.size(); i += 5) {
            poses.add(points.get(i).point());
            Pose2d[] poseArr = poses.toArray(new Pose2d[0]);
            Logger.recordOutput("Pursuiter/ Test 1: load L3.traj", poseArr);
        }

        PathPoint testLook = FastMath.closestPointWithThreshold(0.35f, points.get(0), points);

        Logger.recordOutput("Pursuiter/ Test 1: Lookahead", testLook.point());
    }

    public static void autoTestPeriodic() {

            Swerve.setChassisSpeeds(L3.update(Swerve::getPose));   
            Logger.recordOutput("Pursuiter/ Test 2: Lookahead", L3.getLookAhead().point());
            Logger.recordOutput("Pursuiter/ Test 2: Curr", L3.getCurrentPoint().point());
    }
}
