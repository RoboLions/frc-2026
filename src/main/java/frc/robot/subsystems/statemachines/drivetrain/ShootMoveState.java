import frc.robot.subsystems.interfaces.Sotm;
public class ShootMoveState extends State {
    @Override
    public void build() {
        addTransition(
            new Transition(
                () -> {
                  return RobotMap.driverController.getBButtonPressed();
                },
                DrivetrainStateMachine.teleopState));  
        addTransition(
            new Transition(
                () -> {
                return RobotMap.driverController.getLeftTriggerAxis() > 0.25;
                },
                ScoringStateMachine.alignState));
         
    }

    @Override
    public void init(State prevState) {}  

    @Override
    public void execute() {
        if (CycleState.isPass) {
            if (Swerve.getPose().getY() > 4.0) {
                Swerve.TeleopDriveFacePose(Constants.FIELD.PASS_UPPER.toTranslation2d(), Rotation2d.fromDegrees(180.0), 1.5);
            } else {
                Swerve.TeleopDriveFacePose(Constants.FIELD.PASS_LOWER.toTranslation2d(), Rotation2d.fromDegrees(180.0), 1.5);
            }

            return;
        }

        Swerve.TeleopDriveFacePose(Sotm.getLeadTarget(Constants.FIELD.HUB_POSE),Rotation2d.fromDegrees(180.0), 1.5); 
    }   
    
    @Override
    public void exit(State nextState) {}
}
