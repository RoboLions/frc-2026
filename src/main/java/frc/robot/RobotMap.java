package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.swerve.Swerve;
import frc.robot.subsystems.statemachines.drivetrain.DrivetrainStateMachine;
import frc.robot.subsystems.statemachines.scoring.ScoringStateMachine;

public class RobotMap {

  private static final PowerDistribution PDP = new PowerDistribution();

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
  }

  public static void subsystemPeriodics() {
    Swerve.periodic();
    Limelight.periodic();

    if (driverController.getXButtonPressed()) {
      Swerve.zeroGyro();
    }

    Logger.recordOutput("PDP TOTAL", PDP.getTotalCurrent());
  }
}

// initiate bomb sequence
// - jai patel