package frc.robot.subsystems.statemachines.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotMap;
import frc.robot.lib.statemachine.State;
import frc.robot.lib.statemachine.Transition;
import frc.robot.subsystems.interfaces.swerve.Swerve;


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
    public void init(State prevState) {}  

    @Override
    public void execute() {
        Swerve.driveToPoint(new Pose2d(new Translation2d(2.25, 6.5), Rotation2d.fromDegrees(-45)), 0.75);
    }   
    
    @Override
        public void exit(State nextState) { 
    }
}
