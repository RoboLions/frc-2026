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

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }
    
    private Supplier<AutoRoutine> simulationRoutine() {
        AutoRoutine routine = autoFactory.newRoutine("SIMULATION AUTO (Depot_2Trip_Climb)");

        return () -> {
            AutoTrajectory DEPOT_MID1_STOP = routine.trajectory("DEPOT_MID1_STOP");

            DEPOT_MID1_STOP.atTime(0.25).onTrue(AutoCommands.simulationSetTurretTrack());

            DEPOT_MID1_STOP.atTranslation("2", 0.5)
                   .onTrue(AutoCommands.simulationIntakeDown()
                   .alongWith(AutoCommands.simulationIntakeIn()));

            DEPOT_MID1_STOP.atTranslation("3", 0.5)
                   .onTrue(AutoCommands.simulationIntakeUp()
                   .alongWith(AutoCommands.simulationIntakeStop()));

            DEPOT_MID1_STOP.atTranslation("7", 0.5)
                   .onTrue(AutoCommands.simulationIntakeDown()
                   .alongWith(AutoCommands.simulationIntakeIn()));

            DEPOT_MID1_STOP.atTranslation("9", 0.5)
                   .onTrue(AutoCommands.simulationIntakeUp()
                   .alongWith(AutoCommands.simulationIntakeStop()));

            AutoTrajectory SHOT_MID2_STOP = routine.trajectory("SHOT_MID2_STOP");

            SHOT_MID2_STOP.atTranslation("3", 0.5)
                   .onTrue(AutoCommands.simulationIntakeDown()
                   .alongWith(AutoCommands.simulationIntakeIn()));

            SHOT_MID2_STOP.atTranslation("6", 0.5)
                   .onTrue(AutoCommands.simulationIntakeUp()
                   .alongWith(AutoCommands.simulationIntakeStop()));

            SHOT_MID2_STOP.atPose(SHOT_MID2_STOP.getFinalPose().get(), 0.1, 0.05)
                   .onTrue(AutoCommands.PrintItem("within-tolerance"));

            SHOT_MID2_STOP.done().onTrue(AutoCommands.SwerveStop().andThen(AutoCommands.PrintItem("simPath-Done")));
 
            routine.active().onTrue(
                Commands.sequence(
                    DEPOT_MID1_STOP.resetOdometry(),
                    DEPOT_MID1_STOP.cmd(),
                    AutoCommands.waitAndStopSwerve(2.5),
                    SHOT_MID2_STOP.cmd()
                )
            );

            return routine;
        };
    }
}
