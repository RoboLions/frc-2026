package frc.robot.subsystems.statemachines.drivetrain;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.swerve.Swerve;

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
        Swerve.teleopDrive();
    }   
    
    @Override
        public void exit(State nextState) { 
    }
}
