package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.auto.AutoSubsystem;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.interfaces.Intake;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.interfaces.Turret;
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
    Turret.init();
    Intake.init();

    //THEN STATEMACHINES
    drivetrainStateMachine.enable();
    scoringStateMachine.enable();

    drivetrainStateMachine.setCurrentState(DrivetrainStateMachine.teleopState);
    scoringStateMachine.setCurrentState
    (ScoringStateMachine.idleState);;

    scheduleAuto();
  }

  public static void subsystemPeriodics() {
    Swerve.periodic();
    Limelight.periodic();

    if (Robot.isSimulation() && driverController.getXButtonPressed()) {
      FuelSim.getInstance().clearFuel();
    }

    if (driverController.getXButtonPressed()) {
      Swerve.zeroGyro();
    }
  }

  public static void scheduleAuto() {
    autoSubsystem.scheduleAuto();
  }

  public static void configureFuelSim() {
    FuelSim instance = FuelSim.getInstance();
    instance.spawnStartingFuel();
    instance.registerRobot(
            0.25,
            0.25,
            0.1,
            Swerve::getPose,
            Swerve::getFieldSpeeds);
    instance.registerIntake(
            0.3429,
            0.8429,
            -0.3429,
            0.3429);

    instance.start();

    SmartDashboard.putData(Commands.runOnce(() -> {
                FuelSim.getInstance().clearFuel();
                FuelSim.getInstance().spawnStartingFuel();
            })
            .withName("Reset Fuel")
            .ignoringDisable(true));
  }
}

// initiate bomb sequence