package frc.robot.subsystems.statemachines.drivetrain;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.swerve.Swerve;
import frc.robot.subsystems.statemachines.scoring.CycleState;
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
        if (RobotMap.scoringStateMachine.getCurrentState().equals(ScoringStateMachine.cycleState) && CycleState.isPass) {
            Swerve.teleopDriveSlewed(0.5);
            return;
        }

        Swerve.teleopDrive();
    }   
    
    @Override
    public void exit(State nextState) {}
}