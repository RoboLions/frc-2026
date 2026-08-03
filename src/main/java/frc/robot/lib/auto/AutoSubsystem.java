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

    private final double SHOOT_TIMEOUT_1 = 3.25; // time to empty the hopper
    private final double SHOOT_TIMEOUT_2 = 2.75; // time to empty the hopper

    public AutoSubsystem(AutoFactory createAutoFactory) {
        autoFactory = createAutoFactory;
        autoChooser = new AutoChooser();

        autoChooser.addRoutine("Left-Trench 2P", left2TrenchTripONLY());
        autoChooser.addRoutine("Right-Trench 2P", right2TrenchTripONLY());

        autoChooser.addRoutine("Left-Delay 2P", leftDELAY2Trip());
        autoChooser.addRoutine("Right-Delay 2P", rightDELAY2Trip());

        autoChooser.addRoutine("Left-Bump 2P", leftBump2Trip());
        autoChooser.addRoutine("Right-Bump 2P", rightBump2Trip());

        autoChooser.addRoutine("leftBump2Tripdelay",leftBump2Tripdelay());

        autoChooser.addRoutine("HubPullBack" , Hub_PullBack());

        autoChooser.addRoutine("LeftBumpHub" , Left_Bump_Hub());
        autoChooser.addRoutine("RightBumpHub", Right_Bump_Hub());

        autoChooser.addRoutine("LeftTrenchDelay",left2TrenchTripDELAY());
        autoChooser.addRoutine("RightTrenchDelay",right2TrenchTripDELAY());

        SmartDashboard.putData("AutoChooser", autoChooser);
    }

    public void scheduleAuto() {
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());
    }

    private Supplier<AutoRoutine> left2TrenchTripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("Left 2 PIECE");

        AutoTrajectory L1 = routine.trajectory("L1");
            L1.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));

        AutoTrajectory L2 = routine.trajectory("L2");
            L2.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L2.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));

        AutoTrajectory L3 = routine.trajectory("L3");
            L3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

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

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L3.cmd()));

            return routine;
        };
    }

    private Supplier<AutoRoutine> right2TrenchTripONLY() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE");

        AutoTrajectory R1 = routine.trajectory("L1").mirrorY();
            R1.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R1.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            R1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.001));

        AutoTrajectory R2 = routine.trajectory("L2").mirrorY();
            R2.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R2.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            R2.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.001));

        AutoTrajectory R3 = routine.trajectory("L3").mirrorY();
            R3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    R1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    R2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),
                    
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

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
                    Commands.waitSeconds(2.5),

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    L1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2)));

            return routine;
        };
    }

    private Supplier<AutoRoutine> rightDELAY2Trip() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE DELAY");

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
                    Commands.waitSeconds(2.5),

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    R1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    R2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2)));

            return routine;
        };
    }

    private Supplier<AutoRoutine> leftBump2Trip() {
        AutoRoutine routine = autoFactory.newRoutine("LEFT 2 PIECE BUMP");

        AutoTrajectory L1 = routine.trajectory("L1_Bump");
            L1.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atTime("Intake_ROLLERS_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));


        AutoTrajectory L2 = routine.trajectory("L2_Bump");
            L2.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atTime("Intake_ROLLERS_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L2.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));

        AutoTrajectory L3 = routine.trajectory("L3_Bump");
            L3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));


        return () -> {
            routine.active().onTrue(
                Commands.sequence(

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    L1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),
                        
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                        
                    L3.cmd()));;

            return routine;
        };
    }

    private Supplier<AutoRoutine> rightBump2Trip() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE BUMP");

        AutoTrajectory L1 = routine.trajectory("L1_Bump").mirrorY();
            L1.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        AutoTrajectory L2 = routine.trajectory("L2_Bump").mirrorY();
            L2.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory L3 = routine.trajectory("L3_Bump").mirrorY();
            L3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
                

        return () -> {
            routine.active().onTrue(
                Commands.sequence(

                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    L1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),
                        
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                        
                    L3.cmd()));

            return routine;
        };
    }
    

    private Supplier<AutoRoutine> leftBump2Tripdelay() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE BUMP");

        AutoTrajectory L1 = routine.trajectory("L1_Bump");
            L1.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));


        AutoTrajectory L2 = routine.trajectory("L2_Bump");
            L2.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atPose("Intake_ROLLERS_STOP", 0.5, 1)
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));

        AutoTrajectory L3 = routine.trajectory("L3_Bump");
            L3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
                

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(2.5),


                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    L1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),
                        
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                        
                    L3.cmd()));

            return routine;
        };

    }

    private Supplier<AutoRoutine> Hub_PullBack() {
        AutoRoutine routine = autoFactory.newRoutine("HUB PULL BACK");
        AutoTrajectory L1 = routine.trajectory("L1_Hub");
            
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));
            L1.atTime("INTAKE_OUT")
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.0025));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
        
                    
                    L1.cmd(),
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(20)));
                        
                    

            return routine;
        };

    }






    private Supplier<AutoRoutine> Left_Bump_Hub() {
        AutoRoutine routine = autoFactory.newRoutine(":LEFT HUB PULL BUMP");
        AutoTrajectory L1 = routine.trajectory("L1_BumpHub");
            
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.03));
            L1.atTime("INTAKE_OUT")
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.0025));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
        
                    
                    L1.cmd(),
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(20)));
                        
                    

            return routine;
        };

    }
    private Supplier<AutoRoutine> Right_Bump_Hub() {
        AutoRoutine routine = autoFactory.newRoutine(":RIGHT HUB PULL BUMP");
        AutoTrajectory L1 = routine.trajectory("L1_BumpHub").mirrorY();
            
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.03));
            L1.atTime("INTAKE_OUT")
                .onTrue(AutoCommands.intakeOutOnly()
                .withTimeout(0.0025));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
        
                    
                    L1.cmd(),
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(20)));
                        
                    

            return routine;
        };
    };
    
        private Supplier<AutoRoutine> left2TrenchTripDELAY() {
        AutoRoutine routine = autoFactory.newRoutine("Left 2 PIECE");

        AutoTrajectory L1 = routine.trajectory("L1");
            L1.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L1.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));

        AutoTrajectory L2 = routine.trajectory("L2");
            L2.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            L2.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            L2.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.0025));

        AutoTrajectory L3 = routine.trajectory("L3");
            L3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(2.5),
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.001),
                    
                    L1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    L3.cmd()));

            return routine;
        };

    }
    
    private Supplier<AutoRoutine> right2TrenchTripDELAY() {
        AutoRoutine routine = autoFactory.newRoutine("RIGHT 2 PIECE");

        AutoTrajectory R1 = routine.trajectory("L1").mirrorY();
            R1.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R1.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            R1.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.001));

        AutoTrajectory R2 = routine.trajectory("L2").mirrorY();
            R2.atTime("INTAKE")
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));
            R2.atTime("INTAKE_STOP")
                .onTrue(AutoCommands.intakeStop()
                .withTimeout(0.001));
            R2.atTime("REV_SHOT")
                .onTrue(AutoCommands.setShooter()
                .withTimeout(0.001));

        AutoTrajectory R3 = routine.trajectory("L3").mirrorY();
            R3.atPose("INTAKE", 0.5, 1)
                .onTrue(AutoCommands.intakeOutRollersIn()
                .withTimeout(0.001));

        return () -> {
            routine.active().onTrue(
                Commands.sequence(
                    Commands.waitSeconds(2.5),
                    AutoCommands.intakeZeroPosition()
                        .withTimeout(0.001),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),
                    
                    R1.cmd(),

                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_1),

                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    R2.cmd(),
                    
                    AutoCommands.shootSequenceWithRamp()
                        .withTimeout(SHOOT_TIMEOUT_2),
                    
                    AutoCommands.feedStop()
                        .withTimeout(0.001),
                    
                    AutoCommands.idleShooter()
                        .withTimeout(0.0025),

                    R3.cmd()));

            return routine;
        };
    }
}




