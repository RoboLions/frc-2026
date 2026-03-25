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
        autoChooser.addRoutine("Right 2 Piece ONLY", right2TripONLY());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2TripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory L1 = routine.trajectory("L1");
            L1.atPose("Intake_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.01));
            L1.atPose("ROLLERS_IN", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N1 = routine.trajectory("NL1");
            N1.atPose("INTAKE_MID", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.intakeOutOnly())
                .alongWith(AutoCommands.idleShooter())
                .withTimeout(0.01));
            N1.atPose("SHOOT_RAMP", 1, 1)
                .onTrue(AutoCommands.setShooterAndTrackHub()
                .until(N1.done()));

        AutoTrajectory L2 = routine.trajectory("L2");
            L2.atPose("INTAKE_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.01));
            L2.atPose("ROLLERS_IN", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N2 = routine.trajectory("NL2");
            N2.atPose("INTAKE_MID_TURRET", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.setShooterAndTrackHub())
                .alongWith(AutoCommands.intakeOutOnly())
                .alongWith(AutoCommands.feedStop())
                .until(N2.done()));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.001),
                    
                    L1.cmd(),
                    N1.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.01),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeRollersIn())
                        .alongWith(AutoCommands.intakeMidOnly()
                        .beforeStarting(Commands.waitSeconds(2)))
                        .withTimeout(3.5),

                    AutoCommands.idleShooter()
                        .alongWith(AutoCommands.feedStop())
                        .withTimeout(0.01),

                    L2.cmd(),
                    N2.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.1),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeRollersIn())
                        .alongWith(AutoCommands.intakeMidOnly()
                        .beforeStarting(Commands.waitSeconds(2)))
                        .withTimeout(3.5)
                )
            );

            return routine;
        };
    }

    private Supplier<AutoRoutine> right2TripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE");

        AutoTrajectory R1 = routine.trajectory("R1");
            R1.atPose("Intake_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.01));
            R1.atPose("ROLLERS_IN", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N1 = routine.trajectory("NR1");
            N1.atPose("INTAKE_MID", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.intakeOutOnly())
                .alongWith(AutoCommands.idleShooter())
                .withTimeout(0.01));
            N1.atPose("SHOOT_RAMP", 1, 1)
                .onTrue(AutoCommands.setShooterAndTrackHub()
                .until(N1.done()));

        AutoTrajectory R2 = routine.trajectory("R2");
            R2.atPose("INTAKE_OUT", 1, 1)
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.01));
            R2.atPose("ROLLERS_IN", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.01));

        AutoTrajectory N2 = routine.trajectory("NR2");
            N2.atPose("INTAKE_MID_TURRET", 1, 1)
                .onTrue(AutoCommands.intakeMidRollersStop()
                .alongWith(AutoCommands.setShooterAndTrackHub())
                .alongWith(AutoCommands.intakeOutOnly())
                .alongWith(AutoCommands.feedStop())
                .until(N2.done()));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.001),
                    
                    R1.cmd(),
                    N1.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.01),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeRollersIn())
                        .alongWith(AutoCommands.intakeMidOnly()
                        .beforeStarting(Commands.waitSeconds(2)))
                        .withTimeout(3.5),

                    AutoCommands.idleShooter()
                        .alongWith(AutoCommands.feedStop())
                        .withTimeout(0.01),

                    R2.cmd(),
                    N2.cmd(),

                    AutoCommands.SwerveStop()
                        .withTimeout(0.1),

                    AutoCommands.setShooterAndTrackHub()
                        .alongWith(AutoCommands.feedIn())
                        .alongWith(AutoCommands.intakeRollersIn())
                        .alongWith(AutoCommands.intakeMidOnly()
                        .beforeStarting(Commands.waitSeconds(2)))
                        .withTimeout(3.5)
                )
            );

            return routine;
        };
    }
}
