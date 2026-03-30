package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.auto.AutoSubsystem;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.swerve.Swerve;
import frc.robot.subsystems.statemachines.drivetrain.DrivetrainStateMachine;
import frc.robot.subsystems.statemachines.scoring.ScoringStateMachine;

public class RobotMap {

  private static final AutoSubsystem autoSubsystem = new AutoSubsystem(Swerve.createAutoFactory());

  /* state machine instances */
  public static final ScoringStateMachine scoringStateMachine = new ScoringStateMachine();
  public static final DrivetrainStateMachine drivetrainStateMachine = new DrivetrainStateMachine();

  /* Xbox controllers */
  public static final XboxController manipulatorController = new XboxController(1);
  public static final XboxController driverController = new XboxController(0);

  /* Auto objects */


  public static void init() {
    //FIRST SUBSYSTEMS
    Swerve.init();
    Limelight.init();
    Shooter.init();
    Intake.init();

    //THEN STATEMACHINES
    drivetrainStateMachine.enable();
    scoringStateMachine.enable();

    drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.teleopState);
    scoringStateMachine.setCurrentState(ScoringStateMachine.idleState);;

    scheduleAuto();
  }

  public static void subsystemPeriodics() {
    Swerve.periodic();
    Limelight.periodic();

    if (driverController.getXButtonPressed()) {
      Swerve.zeroGyro();
    }
  }

  public static void scheduleAuto() {
    autoSubsystem.scheduleAuto();
  }
}

// initiate bomb sequence
// - jai patel