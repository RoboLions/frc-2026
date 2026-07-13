package frc.robot.lib.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.auto.Pursuiter.PursuitAutoFactory;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;
import frc.robot.lib.auto.Pursuiter.helpers.FieldMap;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class AutoSubsystem {

    private static SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    private static PursuitProfile profile = new PursuitProfile(
        0.25, 
        15, 
        Units.Meters.of(0.3), 
        new PIDController(1.0, 0, 0),
        new PIDController(5.0, 0, 0),
        new PIDController(10.0, 0, 0),
        true);
    private static PursuitAutoFactory autoFactory = new PursuitAutoFactory(
        Swerve::getPose, Swerve::setFieldChassisSpeeds, profile);

    public static void init() {
        autoFactory.addEvent(
            "INTAKE", Commands.runOnce(() -> System.out.println("INTAKE!!!!!!!!!")));
        autoFactory.addEvent(
            "INTAKE_STOP", Commands.runOnce(() -> System.out.println("INTAKE STOPPP!!!!!!!!!")));
        autoFactory.addEvent(
            "REV_SHOT", Commands.runOnce(() -> System.out.println("REV SHOT!!!!!!!!!")));

        autoChooser.addOption("test 1", testCommand());
        autoChooser.addOption("nothing", new Command() {});
        SmartDashboard.putData(autoChooser);
    }

    public static void autoInit() {
        Swerve.resetPose(new Pose2d(new Translation2d(15, 0), new Rotation2d()));
        CommandScheduler.getInstance().schedule(autoChooser.getSelected());
    }

    private static Command testCommand() {
        return autoFactory.followPath("L1.traj", true, true, FieldMap.center)
                .andThen(autoFactory.followPath("L2.traj", true, true, FieldMap.center))
                .andThen(autoFactory.followPath("L3.traj", true, true, FieldMap.center));
    }
}
