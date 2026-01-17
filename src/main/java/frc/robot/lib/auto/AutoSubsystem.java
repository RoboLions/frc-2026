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

        //MANUAL EVENT MARKERS
        Pose2d startTurret1 = new Pose2d(new Translation2d(0.6369214057922363, 6.62243747711181), new Rotation2d(-1.269176983105683));
        Pose2d stopTurret1 = new Pose2d(new Translation2d(3.599396228790283, 7.41448783874511), new Rotation2d(0));

        return () -> {
            AutoRoutine routine = autoFactory.newRoutine("simulation");

            AutoTrajectory simPath = routine.trajectory("FastPath");
 
            routine.active().onTrue(
                Commands.sequence(
                    simPath.resetOdometry(),
                    simPath.cmd()
                )
            );

            simPath.atTime(0.5).onTrue(AutoCommnads.setTurretTrack());
            
            simPath.atPose(simPath.getFinalPose().get(), 0.1, 0.05)
                   .onTrue(AutoCommnads.PrintItem("within-tolerance"));

            simPath.done().onTrue(AutoCommnads.SwerveStop().andThen(AutoCommnads.PrintItem("simPath-Done")));

            return routine;
        };
    }
}
