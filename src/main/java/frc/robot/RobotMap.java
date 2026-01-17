package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.auto.AutoSubsystem;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.statemachines.scoring.ScoringStateMachine;
import frc.robot.subsystems.swerve.Swerve;

public class RobotMap {

  private static final AutoSubsystem autoSubsystem = new AutoSubsystem(Swerve.createAutoFactory());

  /* state machine instances */
  // public static final DrivetrainStateMachine drivetrainStateMachine = new DrivetrainStateMachine();
  public static final ScoringStateMachine scoringStateMachine = new ScoringStateMachine();

  /* Xbox controllers */
  public static final XboxController manipulatorController = new XboxController(1);
  public static final XboxController driverController = new XboxController(0);

  /* Auto objects */


  public static void init() {
    Shooter.init();
    Limelight.init();
    Swerve.init();

    // drivetrainStateMachine.init();
  }

  public static void subsystemPeriodics() {
    Swerve.periodic();

    if (DriverStation.isTeleopEnabled()) {
      Swerve.simulationDrive();
    }
  }

  public static void simulationAuto() {
    autoSubsystem.scheduleAutoSimulation();
  }
}
// initiate bomb sequence
