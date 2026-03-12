package frc.robot.lib.auto;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.interfaces.Turret.SimulationObjects;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class AutoCommands {

    public static Trigger exampleTrigger() {
        return new Trigger(() -> false);
    }

    public static Command exampleCommand() {
        return Commands.runOnce(() -> System.out.println("Hello World!"));
    }
    
    public static Command SwerveStop() {
        return Commands.runOnce(() -> Swerve.zeroCommand());
    }

    public static Command PrintItem(String string) {
        return Commands.runOnce(() -> System.out.println(string));
    }

    public static Command setTurretTrack() {
        return Commands.run(() -> Turret.turretTrackHub());
    }

    public static Command setTurretToZero() {
        return Commands.run(() -> Turret.setAzimuthZero()); 
    }

    public static Command startShooter() {
        return Commands.run(() ->  Shooter.idlerShooter());
    }

    public static Command shootAndTrackHub() {
        return Commands.run(() ->  Shooter.setShootSpeed(SimulationObjects.totalShotVelocity))
                       .alongWith(Commands.run(() -> Turret.turretTrackHub()));
    }

    public static Command intakeOutRollersIn() {
        return Commands.run(() -> Intake.allRollersIn())
                       .alongWith(Commands.run(() -> Intake.intakeDown()));
    }

    public static Command intakeMidRollersStop() {
        return Commands.run(() -> Intake.allRollersStop())
                       .alongWith(Commands.run(() -> Intake.intakeMid()));
    }

    public static Command feedIn() {
        return Commands.run(() -> Intake.allRollersIn());
    }

    public static BooleanSupplier timerAt(double time, Timer timer) {
        return (() -> true);
    }

    //all 5000 log item commands
    public static Command LogItem(String key, String data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Double data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Boolean data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Integer data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Pose2d data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Translation2d data) {
        return Commands.runOnce(() -> Logger.recordOutput(key, data));
    }

    public static Command LogItem(String key, Command command) {
        return Commands.runOnce(() -> Logger.recordOutput(key, command.getName()));
    }

    public static Command LogItem(String key, Timer timer) {
        return Commands.runOnce(() -> Logger.recordOutput(key, timer.get()));
    }

    public static Command waitAndStopSwerve(double waitSeconds) {
        return Commands.waitSeconds(waitSeconds).alongWith(Commands.runOnce(() -> Swerve.zeroCommand()));

    }
}
