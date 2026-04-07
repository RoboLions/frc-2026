package frc.robot.subsystems.statemachines.drivetrain;

import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.swerve.Swerve;


public class BrakeState extends State {
    @Override
    public void build() {
        addTransition(
            new Transition(
                () -> {
                  return RobotMap.driverController.getBButtonPressed() || RobotMap.driverController.getRightBumperButtonPressed();
                },
                DrivetrainStateMachine.teleopState));    
        addTransition(
            new Transition(
                () -> {
                  return RobotMap.driverController.getLeftTriggerAxis() > 0.25;
                },
                DrivetrainStateMachine.alignState));   
    }

    @Override
    public void init(State prevState) {
        Swerve.brakeX();
    }  

    @Override
    public void execute() {}   
    
    @Override
        public void exit(State nextState) { 
    }
}
