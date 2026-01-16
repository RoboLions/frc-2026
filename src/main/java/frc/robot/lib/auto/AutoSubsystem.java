package frc.robot.lib.auto;

import java.util.function.Supplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

public class AutoSubsystem {
    private AutoFactory autoFactory;
    private AutoChooser autoChooser;

    public AutoSubsystem(AutoFactory createAutoFactory) {
        autoFactory = createAutoFactory;
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("simulation", simulationRoutine());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAutoSimulation() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> simulationRoutine() {
        return () -> {
            AutoRoutine routine = autoFactory.newRoutine("simulation");

            AutoTrajectory simPath = routine.trajectory("longSimulationPath");
 
            routine.active().onTrue(
                Commands.sequence(
                    simPath.resetOdometry(),
                    simPath.cmd()
                )
            );

            simPath.atPose(new Pose2d(new Translation2d(1.356, 6.94597053527832), new Rotation2d(-0.73866)), 0.2, 3)
                   .onTrue(AutoCommnads.PrintItem("within-tolerance, shooting turret"));
                   
            simPath.atPose(simPath.getFinalPose().get(), 0.1, 0.05)
                   .onTrue(AutoCommnads.PrintItem("within-tolerance"));

            simPath.done().onTrue(AutoCommnads.SwerveStop().andThen(AutoCommnads.PrintItem("simPath-Done")));

            return routine;
        };
    }
}
