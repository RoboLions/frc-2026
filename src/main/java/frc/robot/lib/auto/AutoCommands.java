package frc.robot.lib.auto;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Shooter;
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

    public static Command SwerveFaceHUB() {
        return Commands.run(() -> Swerve.facePose(Constants.FIELD.HUB_POSE, Rotation2d.fromDegrees(180)));
    }

    public static Command PrintItem(String string) {
        return Commands.runOnce(() -> System.out.println(string));
    }

    public static Command idleShooter() {
        return Commands.run(() ->  Shooter.idlerShooter());
    }

    public static Command setShooter() {
        return Commands.run(() ->  Shooter.interpolateAndShoot(Swerve.getPose().getTranslation().getDistance(Constants.FIELD.HUB_POSE)));
    }

    public static Command shootSequenceWithRamp() {
        return Commands.sequence(AutoCommands.SwerveFaceHUB()
                        .alongWith(AutoCommands.setShooter())
                        .withTimeout(0.5),
                
                    AutoCommands.feedIn()
                        .alongWith(AutoCommands.SwerveFaceHUB())
                        .alongWith(AutoCommands.setShooter())
                        .withTimeout(0.75),

                    AutoCommands.intakeZeroPosition()
                        .alongWith(AutoCommands.SwerveFaceHUB())
                        .alongWith(AutoCommands.setShooter())
                        .alongWith(AutoCommands.intakeZeroPosition())
                        .withTimeout(2.0));
    }

        public static Command shootSequenceNoRamp() {
        return Commands.sequence(AutoCommands.SwerveStop()
                        .alongWith(AutoCommands.setShooter())
                        .alongWith(AutoCommands.feedIn()));
    }

    public static Command intakeOutRollersIn() {
        return Commands.run(() -> Intake.intake())
                       .alongWith(Commands.run(() -> Intake.intakeDown()));
    }

    public static Command intakeOutOnly() {
        return Commands.run(() -> Intake.intakeDown());
    }

    public static Command intakeStop() {
        return Commands.run(() -> Intake.stopIntake());
    }

    public static Command intakeMidOnly() {
        return Commands.run(() -> Intake.intakeMid());
    }

    public static Command intakeRollersIn() {
        return Commands.run(() -> Intake.intake());
    }

    public static Command intakeMidRollersStop() {
        return Commands.run(() -> Intake.stopIntake())
                       .alongWith(Commands.run(() -> Intake.intakeMid()));
    }

    public static Command feedIn() {
        return Commands.run(() -> Intake.FeedIn())
                       .alongWith(Commands.run(() -> Intake.intake()))
                       .alongWith(Commands.run(() -> Intake.IndexIn()));
    }

    public static Command feedStop() {
        return Commands.run(() -> Intake.stopFeed())
                        .alongWith(Commands.run(() -> Intake.stopIndex())
                        .alongWith(Commands.run(() -> Intake.stopIntake())));
    }

    public static Command intakeZeroPosition() {
        return Commands.run(() -> Intake.intakeUp());
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
