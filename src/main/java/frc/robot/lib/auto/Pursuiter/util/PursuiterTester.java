package frc.robot.lib.auto.Pursuiter.util;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class PursuiterTester {

    static PursuitPath L3 = new PursuitPath(
            0.5, 
            10, 
            Units.Meters.of(0.3),
            new PIDController(1.0, 0, 0),
            new PIDController(7.5, 0, 0), 
            "L1.traj");

    public static void initTest() {
        testLoadInit();
        Swerve.resetPose(L3.getCurrentPoint().point());
    }
    
    public static void testLoadInit() {
        List<PathPoint> points = PathLoader.loadSample("L1.traj");
        List<Pose2d> poses = new ArrayList<Pose2d>();

        for (int i = 0; i < points.size(); i += 5) {
            poses.add(points.get(i).point());
            Pose2d[] poseArr = poses.toArray(new Pose2d[0]);
            Logger.recordOutput("Pursuiter/ Test 1: load L1.traj", poseArr);
        }
    }

    public static void autoTestPeriodic() {
            Swerve.setFieldChassisSpeeds(L3.update(Swerve::getPose));   
            Logger.recordOutput("Pursuiter/ Test 2: Lookahead", L3.getLookAhead().point());
            Logger.recordOutput("Pursuiter/ Test 2: LookaheadPt Index", L3.getLookAhead().pointIndex());
            Logger.recordOutput("Pursuiter/ Test 2: Curr", L3.getCurrentPoint().point());
            Logger.recordOutput("Pursuiter/ Test 2: CurrPt Index", L3.getCurrentPoint().pointIndex());
    }
}
