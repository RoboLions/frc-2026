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

        autoChooser.addRoutine("simulation", simulationRoutine());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAutoSimulation() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> simulationRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("SIMULATION AUTO (Depot_2Trip_Climb)");

        return () -> {
            AutoTrajectory simPath = routine.trajectory("Depot_2Trip_Climb");
 
            routine.active().onTrue(
                Commands.sequence(
                    simPath.resetOdometry(),
                    simPath.cmd()
                )
            );

            simPath.atTime(0.25).onTrue(AutoCommnads.simulationSetTurretTrack());

            simPath.atTranslation("2", 0.5)
                   .onTrue(AutoCommnads.simulationIntakeDown()
                   .alongWith(AutoCommnads.simulationIntakeIn()));

            simPath.atTranslation("3", 0.5)
                   .onTrue(AutoCommnads.simulationIntakeUp()
                   .alongWith(AutoCommnads.simulationIntakeStop()));

            simPath.atTranslation("7", 0.5)
                   .onTrue(AutoCommnads.simulationIntakeDown()
                   .alongWith(AutoCommnads.simulationIntakeIn()));

            simPath.atTranslation("9", 0.5)
                   .onTrue(AutoCommnads.simulationIntakeUp()
                   .alongWith(AutoCommnads.simulationIntakeStop()));

            simPath.atTranslation("12", 0.5)
                   .onTrue(AutoCommnads.simulationIntakeDown()
                   .alongWith(AutoCommnads.simulationIntakeIn()));       

            simPath.atPose(simPath.getFinalPose().get(), 0.1, 0.05)
                   .onTrue(AutoCommnads.PrintItem("within-tolerance"));

            simPath.done().onTrue(AutoCommnads.SwerveStop().andThen(AutoCommnads.PrintItem("simPath-Done")));

            return routine;
        };
    }
}
