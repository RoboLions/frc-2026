package frc.robot.lib.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.auto.Pursuiter.PursuitAutoFactory;
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
    private static PursuitAutoFactory autoFactory = new PursuitAutoFactory(
        Swerve::getPose, Swerve::setFieldChassisSpeeds, profile);

    public static void init() {
    }

    public static void autoInit() {
        CommandScheduler.getInstance().schedule(autoFactory.followPath("L3.traj"));
    }

    public static void autoPeriodic() {}
}
