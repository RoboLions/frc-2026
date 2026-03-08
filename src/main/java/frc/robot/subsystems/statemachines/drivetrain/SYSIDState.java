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
         * Joystick RightPOV = quasistatic forward
         * Joystick LeftPOV = quasistatic reverse
         * Joystick UpPOV = dynamic forward
         * Joystick DownPOV = dyanmic reverse
         */

        Trigger RightPOV = new Trigger(() -> RobotMap.driverController.getPOV() == 90);
        RightPOV.whileTrue(Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

        Trigger LeftPOV = new Trigger(() -> RobotMap.driverController.getPOV() == 270);
        LeftPOV.whileTrue(Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        Trigger UpPOV = new Trigger(() -> RobotMap.driverController.getPOV() == 0);
        UpPOV.whileTrue(Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));

        Trigger DownPOV = new Trigger(() -> RobotMap.driverController.getPOV() == 180);
        DownPOV.whileTrue(Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        CommandScheduler.getInstance().run();
    }  
    
    @Override
    public void exit(State nextState) { 
        Commands.runOnce(SignalLogger::stop);
        CommandScheduler.getInstance().cancelAll();
    }
}
