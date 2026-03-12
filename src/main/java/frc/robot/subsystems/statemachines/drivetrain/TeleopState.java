package frc.robot.subsystems.statemachines.drivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.statemachines.scoring.ScoringStateMachine;

public class TeleopState extends State {
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

    }  

    @Override
    public void execute() {
        if (RobotMap.scoringStateMachine.getCurrentState().equals(ScoringStateMachine.cycleState)) {
            Swerve.teleopDrive(0.4);
            return;
        }

        Swerve.teleopDrive();
    }   
    
    @Override
    public void exit(State nextState) {}
}