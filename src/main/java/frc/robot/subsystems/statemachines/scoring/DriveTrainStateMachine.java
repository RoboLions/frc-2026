package frc.robot.subsystems.statemachines.scoring;

public class DriveTrainStateMachine {
  
  public static OuttakeState outtakeState = new OuttakeState(); 
  public static BrakeState brakeState = new BrakeState(); 
  public static IntakeState intakeState = new IntakeState(); 
  public static ShootState shootState = new ShootState();
  public static IdleState idleState = new IdleState(); 


  public DriveTrainStateMachine(){
    outtakeState();
    intakeState();
    brakeState();
    shootState();
    idleState();
  }


  
}
