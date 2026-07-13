package frc.robot.lib.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;

public class AutoSubsystem {

    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private static PursuitProfile profile = new PursuitProfile(
        0.25, 
        15, 
        Units.Meters.of(0.3), 
        new PIDController(1.0, 0, 0),
            new PIDController(4.0, 0, 0),
            new PIDController(10.0, 0, 0));

    public static void init() {
        PursuitPath test = new PursuitPath(profile, "L3.traj");
        Logger.recordOutput("/Pursuit/ test Pose list", test.getEventPoses());
    }

    public static void autoInit() {}

    public static void autoPeriodic() {}
}
