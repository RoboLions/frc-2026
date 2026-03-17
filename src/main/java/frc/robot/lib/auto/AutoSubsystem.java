package frc.robot.lib.auto;

import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class AutoSubsystem {
    public AutoFactory autoFactory;
    private AutoChooser autoChooser;

    public AutoSubsystem(AutoFactory createAutoFactory) {
        autoFactory = createAutoFactory;
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Left 2 Piece ONLY", left2TripONLY());
        autoChooser.addRoutine("TEST", test());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2TripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory L1 = routine.trajectory("L1");
            L1.atPose("Intake_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N1 = routine.trajectory("N1");
            N1.atPose("INTAKE_MID", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.idleShooter())
                .withTimeout(0.01));
            N1.atPose("SHOOT_RAMP", 1, 1)
                .onTrue(AutoCommands.setShooterAndTrackHub()
                .until(N1.done()));

        AutoTrajectory L2 = routine.trajectory("L2");
            L2.atPose("INTAKE_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N2 = routine.trajectory("N2");
            N2.atPose("INTAKE_MID_TURRET", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.setShooterAndTrackHub())
                .alongWith(AutoCommands.feedStop())
                .until(N2.done()));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.01),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(2),

                    AutoCommands.feedStop()
                        .withTimeout(0.01),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.01),
                    
                    L1.cmd(),
                    N1.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.01),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeZeroPosition()
                        .beforeStarting(Commands.waitSeconds(1.75)))
                        .withTimeout(2.75),

                    AutoCommands.idleShooter()
                        .alongWith(AutoCommands.feedStop())
                        .withTimeout(0.01),

                    L2.cmd(),
                    N2.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.1),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeZeroPosition()
                        .beforeStarting(Commands.waitSeconds(1.75)))
                        .withTimeout(2.75)
                )
            );

            return routine;
        };
    }

    private Supplier<AutoRoutine> test() {
        AutoRoutine routine = autoFactory.newRoutine("TEST");

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.setTurretTrack()
                        .alongWith(AutoCommands.setShooter())
                        .withTimeout(0.5),
                    AutoCommands.feedIn(),
                    AutoCommands.SwerveStop()
                )
            );
        
            return routine;
        };
    }
}
