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

    private Supplier<AutoRoutine> simulationRoutine() {
        return () -> {
            AutoRoutine routine = autoFactory.newRoutine("simulation");

            AutoTrajectory simPath = routine.trajectory("Simulation_Path");

            routine.active().onTrue(
                Commands.sequence(
                    simPath.resetOdometry(),
                    simPath.cmd()
                )
            );
            

            return routine;
        };
    }

    public void scheduleAutoSimulation() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }
}
