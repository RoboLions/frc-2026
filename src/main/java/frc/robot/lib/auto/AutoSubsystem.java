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

        autoChooser.addRoutine("Left 2 Piece", left2Piece());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2Piece() {
        AutoRoutine routine = autoFactory.newRoutine("testPath");

        AutoTrajectory LEFT_START_TO_NEUTRAL_ZONE = routine.trajectory("LEFT_START_TO_NEUTRAL_ZONE");
            LEFT_START_TO_NEUTRAL_ZONE.atPose("Intake_OUT", 0.5, 0.5)
                .onTrue(AutoCommands.intakeOutRollersIn());

        AutoTrajectory Neutral_Zone_TO_LEFT_TRENCH = routine.trajectory("Neutral_Zone_TO_LEFT_TRENCH");
            Neutral_Zone_TO_LEFT_TRENCH.atPose("INTAKE_MID", 0.5, 0.5)
                .onTrue(AutoCommands.intakeMidRollersStop())
                .onTrue(AutoCommands.startShooter());

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    LEFT_START_TO_NEUTRAL_ZONE.cmd(),
                    Neutral_Zone_TO_LEFT_TRENCH.cmd(),
                    AutoCommands.SwerveStop()
                        .alongWith(AutoCommands.shootAndTrackHub())
                        .alongWith(Commands.waitSeconds(2))
                )
            );
        
            return routine;
        };
    }
}
