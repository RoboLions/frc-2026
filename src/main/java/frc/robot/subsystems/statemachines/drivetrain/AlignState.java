package frc.robot.subsystems.statemachines.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.swerve.Swerve;


public class AlignState extends State {
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
        Swerve.driveToPoint(new Pose2d(new Translation2d(3,3), new Rotation2d(0)), 1);
    }   
    
    @Override
        public void exit(State nextState) { 
    }
}
