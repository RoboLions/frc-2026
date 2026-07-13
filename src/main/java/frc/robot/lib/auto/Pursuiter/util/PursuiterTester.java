package frc.robot.lib.auto.Pursuiter.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class PursuiterTester {

    static PursuitPath L1 = new PursuitPath(
            0.25, 
            15, 
            Units.Meters.of(0.3),
            new PIDController(1.0, 0, 0),
            new PIDController(3.0, 0, 0),
            new PIDController(10.0, 0, 0), 
            "L1.traj");

    static PursuitPath L2 = new PursuitPath(
            0.25, 
            15, 
            Units.Meters.of(0.3),
            new PIDController(1.0, 0, 0),
            new PIDController(3.0, 0, 0),
            new PIDController(10.0, 0, 0), 
            "L2.traj");

    static PursuitPath L3 = new PursuitPath(
            0.25, 
            15, 
            Units.Meters.of(0.3),
            new PIDController(1.0, 0, 0),
            new PIDController(3.0, 0, 0),
            new PIDController(10.0, 0, 0), 
            "L3.traj");

    public static void initTest() {
        testLoadInit();
        Swerve.resetPose(L1.getCurrentPoint().point());

        L1.append(L2);
        L1.append(L3);
    }
    
    public static void testLoadInit() {
        Logger.recordOutput("Pursuiter/ Test 1: load L1.traj", L1.getVisualizedPath(5));
    }

    public static void autoTestInit() {
        CommandScheduler.getInstance().schedule(L1.toCommand(Swerve::getPose, Swerve::setFieldChassisSpeeds));
    }

    public static void autoTestPeriodic() {
        Logger.recordOutput("Pursuiter/ Test 2: Lookahead", L1.getLookAhead().point());
        Logger.recordOutput("Pursuiter/ Test 1: load L1.traj", L1.getVisualizedPath(5));
        Logger.recordOutput("Pursuiter/ Test 2: LookaheadPt Index", L1.getLookAhead().pointIndex());
        Logger.recordOutput("Pursuiter/ Test 2: Curr", L1.getCurrentPoint().point());
        Logger.recordOutput("Pursuiter/ Test 2: CurrPt Index", L1.getCurrentPoint().pointIndex());
    }
}
