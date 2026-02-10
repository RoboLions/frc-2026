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
    private AutoFactory autoFactory;
    private AutoChooser autoChooser;

    public AutoSubsystem(AutoFactory createAutoFactory) {
        autoFactory = createAutoFactory;
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("testPath", testRoutine());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> testRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("testPath");

        return () -> {
            AutoTrajectory testPath = routine.trajectory("testPath");

            testPath.atTime(0.5).onTrue(AutoCommands.setTurretTrack()
                                               .alongWith(AutoCommands.startShooter()));
            testPath.atTime(1).onTrue(AutoCommands.feedIn());


            routine.active().onTrue(
                Commands.sequence(
                    testPath.cmd(),
                    AutoCommands.SwerveStop()
                )
                
            );
        
            return routine;
        };
    }
}
