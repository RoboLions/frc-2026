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

        autoChooser.addRoutine("Left Trench 2 Piece ONLY", left2TrenchTripONLY());
        autoChooser.addRoutine("Right Trench 2 Piece ONLY", right2TrenchTripONLY());

        autoChooser.addRoutine("Left DELAY 2 Piece", leftDELAY2Trip());
        autoChooser.addRoutine("Right DELAY 2 Piece", rightDELAY2Trip());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2TrenchTripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory L1 = routine.trajectory("L1");
            L1.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        AutoTrajectory NL1 = routine.trajectory("N1");
            NL1.atPose("INTAKE_STOP", 1, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory L2 = routine.trajectory("L2");
            L2.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        AutoTrajectory NL2 = routine.trajectory("N2");
            NL2.atPose("INTAKE_STOP", 1, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory L3 = routine.trajectory("L3");
            L3.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),
                    
                    L1.cmd(),
                    NL1.cmd(),

                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.0),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    L2.cmd(),
                    NL2.cmd(),
                    
                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.0),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    L3.cmd()));

            return routine;
        };
    }

    private Supplier<AutoRoutine> right2TrenchTripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory R1 = routine.trajectory("L1").mirrorY();
            R1.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        AutoTrajectory NR1 = routine.trajectory("N1").mirrorY();
            NR1.atPose("INTAKE_STOP", 1, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory R2 = routine.trajectory("L2").mirrorY();
            R2.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        AutoTrajectory NR2 = routine.trajectory("N2").mirrorY();
            NR2.atPose("INTAKE_STOP", 1, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory R3 = routine.trajectory("L3").mirrorY();
            R3.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),
                    
                    R1.cmd(),
                    NR1.cmd(),

                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.0),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    R2.cmd(),
                    NR2.cmd(),
                    
                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.0),
                    
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    R3.cmd()));

            return routine;
        };
    }

    private Supplier<AutoRoutine> leftDELAY2Trip() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE DELAY");

        AutoTrajectory L1 = routine.trajectory("L1_DELAY");
            L1.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        AutoTrajectory L2 = routine.trajectory("L2_DELAY");
            L2.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(1.0),

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),
                    
                    L1.cmd(),

                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.5),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    L2.cmd(),
                    
                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.5)));

            return routine;
        };
    }

    private Supplier<AutoRoutine> rightDELAY2Trip() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE");

        AutoTrajectory R1 = routine.trajectory("L1_DELAY").mirrorY();
            R1.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R1.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        AutoTrajectory R2 = routine.trajectory("L2_DELAY").mirrorY();
            R2.atPose("INTAKE", 1, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R2.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(1.0),

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),
                    
                    R1.cmd(),

                    // AutoCommands.shootSequenceWithRamp()
                    //     .withTimeout(3.5),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    // AutoCommands.idleShooter()
                    //     .withTimeout(0.001),

                    R2.cmd(),
                    
                    // AutoCommands.shootSequenceWithRamp()
                    //     // .withTimeout(3.5)));

            return routine;
        };
    }
}