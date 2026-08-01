public class ShootMoveState extends State {
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
        if (CycleState.isPass) {
            if (Swerve.getPose().getY() > 4.0) {
                Swerve.TeleopDriveFacePose(Constants.FIELD.PASS_UPPER.toTranslation2d(), Rotation2d.fromDegrees(180.0));
            } else {
                Swerve.TeleopDriveFacePose(Constants.FIELD.PASS_LOWER.toTranslation2d(), Rotation2d.fromDegrees(180.0));
            }

            return;
        }

        Swerve.TeleopDriveFacePose(Constants.FIELD.HUB_POSE, Rotation2d.fromDegrees(180.0));
    }   
    
    @Override
    public void exit(State nextState) {}
}
