package frc.robot.subsystems.statemachines.drivetrain;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.swerve.Swerve;

public class SYSIDState extends State{
    @Override
    public void build() {
        addTransition(
            new Transition(
                () -> {
                  return RobotMap.driverController.getBButtonPressed();
                },
                DrivetrainStateMachine.teleopState));    
    }

    @Override
    public void init(State prevState) {
        Commands.runOnce(SignalLogger::start);
        CommandScheduler.getInstance().run();
    }  

    @Override
    public void execute() {
        /*
         * Make sure to comment out all other idleState transitions in Scoring or you might screw this up.
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic reverse
         * Joystick rightBumper = dynamic forward
         * Joystick leftBumper = dyanmic reverse
         */

        Trigger y = new Trigger(RobotMap.driverController::getYButton);
        y.whileTrue(Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        Trigger a = new Trigger(RobotMap.driverController::getAButton);
        a.whileTrue(Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        Trigger r = new Trigger(RobotMap.driverController::getRightBumperButton);
        r.whileTrue(Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));

        Trigger l = new Trigger(RobotMap.driverController::getLeftBumperButton);
        l.whileTrue(Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));


        CommandScheduler.getInstance().run();
    }  
    
    @Override
    public void exit(State nextState) { 
        Commands.runOnce(SignalLogger::stop);
        CommandScheduler.getInstance().cancelAll();
    }
}
