package frc.robot.lib.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class AutoSubsystem {

    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private static PursuitProfile profile = new PursuitProfile(
        0.25, 
        15, 
        Units.Meters.of(0.3), 
        new PIDController(1.0, 0, 0),
            new PIDController(4.0, 0, 0),
            new PIDController(10.0, 0, 0));

    private static PursuitPath L1 = new PursuitPath(profile, "L1.traj");
    private static PursuitPath L2 = new PursuitPath(profile, "L2.traj");
    private static PursuitPath L3 = new PursuitPath(profile, "L3.traj");

    public static void init() {
        Swerve.resetPose(L1.getCurrentPoint().point());
        Logger.recordOutput("Pursuiter/ Test 1: load L1.traj", L1.getVisualizedPath(5));

        L1.append(L2);
        L1.append(L3);
    }

    public static void autoInit() {
        CommandScheduler.getInstance().schedule(L1.toCommand(Swerve::getPose, Swerve::setFieldChassisSpeeds));
    }

    public static void autoPeriodic() {
        Logger.recordOutput("Pursuiter/ Test 2: Lookahead", L1.getLookAhead().point());
        Logger.recordOutput("Pursuiter/ Test 1: load L1.traj", L1.getVisualizedPath(5));
        Logger.recordOutput("Pursuiter/ Test 2: LookaheadPt Index", L1.getLookAhead().pointIndex());
        Logger.recordOutput("Pursuiter/ Test 2: Curr", L1.getCurrentPoint().point());
        Logger.recordOutput("Pursuiter/ Test 2: CurrPt Index", L1.getCurrentPoint().pointIndex());
    }
}
