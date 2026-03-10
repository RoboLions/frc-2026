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

        // autoChooser.addRoutine("testPath", testRoutine());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2Piece() {
        AutoRoutine routine = autoFactory.newRoutine("testPath");

        AutoTrajectory Neutral_Zone_TO_LEFT_TRENCH = routine.trajectory("Neutral_Zone_TO_LEFT_TRENCH");
            // Neutral_Zone_TO_LEFT_TRENCH.atTime(0.1);
        
        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Neutral_Zone_TO_LEFT_TRENCH.cmd(),
                    AutoCommands.SwerveStop()
                )
            );
        
            return routine;
        };
    }
}
