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

        autoChooser.addRoutine("Left 1 Piece", left1Trip());
        autoChooser.addRoutine("Left 2 Piece", left2TripDepot());
        autoChooser.addRoutine("TEST", test());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

        private Supplier<AutoRoutine> left2TripDepot() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory LEFT_START_TO_NEUTRAL_ZONE = routine.trajectory("LEFT_START_TO_NEUTRAL_ZONE");
            LEFT_START_TO_NEUTRAL_ZONE.atPose("Intake_OUT", 0.5, 0.5)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory Neutral_Zone_TO_LEFT_TRENCH = routine.trajectory("Neutral_Zone_TO_LEFT_TRENCH");
            Neutral_Zone_TO_LEFT_TRENCH.atPose("INTAKE_MID", 0.5, 0.5)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.startShooter())
                .withTimeout(0.01));
            Neutral_Zone_TO_LEFT_TRENCH.atPose("SHOOT_RAMP", 0.5, 0.5)
                .onTrue(AutoCommands.setShooterAndTrackHub()
                .withTimeout(0.01));

        AutoTrajectory LEFT_TRENCH_TO_DEPOT = routine.trajectory("LEFT_TRENCH_TO_DEPOT");
            LEFT_TRENCH_TO_DEPOT.atPose("INTAKE_OUT", 05, 0.5)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(1.5),

                    AutoCommands.feedStop()
                        .withTimeout(0.01),
                    
                    AutoCommands.startShooter()
                        .withTimeout(0.01),
                    
                    LEFT_START_TO_NEUTRAL_ZONE.cmd(),
                    Neutral_Zone_TO_LEFT_TRENCH.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.01),

                    AutoCommands.shootSequenceNoRamp()
                        .withTimeout(2),

                    AutoCommands.feedIn()
                        .withTimeout(2.5),

                    AutoCommands.startShooter()
                        .withTimeout(0.01),

                    LEFT_START_TO_NEUTRAL_ZONE.cmd(),
                    Neutral_Zone_TO_LEFT_TRENCH.cmd(),

                    LEFT_TRENCH_TO_DEPOT.cmd()
                        .alongWith(AutoCommands.shootSequenceNoRamp()
                            .withTimeout(0.01))
                        .alongWith(AutoCommands.feedIn()
                            .withTimeout(0.01)),
                    
                    AutoCommands.SwerveStop()
                )
            );

            return routine;
        };
    }

    private Supplier<AutoRoutine> left1Trip() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 1 PIECE");

        AutoTrajectory LEFT_START_TO_NEUTRAL_ZONE = routine.trajectory("LEFT_START_TO_NEUTRAL_ZONE");
            LEFT_START_TO_NEUTRAL_ZONE.atPose("Intake_OUT", 0.5, 0.5)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory Neutral_Zone_TO_LEFT_TRENCH = routine.trajectory("Neutral_Zone_TO_LEFT_TRENCH");
            Neutral_Zone_TO_LEFT_TRENCH.atPose("INTAKE_MID", 0.5, 0.5)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.startShooter())
                .withTimeout(0.01));
            Neutral_Zone_TO_LEFT_TRENCH.atPose("SHOOT_RAMP", 0.5, 0.5)
                .onTrue(AutoCommands.setShooterAndTrackHub()
                .withTimeout(0.01));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(1.5),

                    AutoCommands.feedStop()
                        .withTimeout(0.01),
                    
                    AutoCommands.startShooter()
                        .withTimeout(0.01),
                    
                    LEFT_START_TO_NEUTRAL_ZONE.cmd(),
                    Neutral_Zone_TO_LEFT_TRENCH.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.01),

                    AutoCommands.shootSequenceNoRamp()
                        .withTimeout(2),

                    AutoCommands.feedIn()
                        .withTimeout(3)
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
