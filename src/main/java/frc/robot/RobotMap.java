package frc.robot;

import java.time.Duration;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.auto.AutoSubsystem;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.interfaces.Turret;
import frc.robot.subsystems.interfaces.Intake;
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
    //FIRST SUBSYSTEMS
    Swerve.init();

    Limelight.init();

    Shooter.init();

    //THEN STATEMACHINES

    // drivetrainStateMachine.init();
  }

  public static void subsystemPeriodics() {
    Swerve.periodic();
    Limelight.periodic();

    if (DriverStation.isTeleopEnabled() && Robot.isSimulation()) {
      Swerve.simulationDrive();

      if (driverController.getXButtonPressed()) {
        FuelSim.getInstance().clearFuel();
      }
    }

    scoringStateMachine.setNextState();
  }

  public static void simulationAuto() {
    autoSubsystem.scheduleAutoSimulation();
  }

  public static void simulateFuelPeriodics() {
    if (scoringStateMachine.getCurrentState().equals(ScoringStateMachine.shootState)) {
      Turret.launchFuel();
    }

    Logger.recordOutput("Fuel Sim/ BLUE SCORE", FuelSim.Hub.BLUE_HUB.getScore()); // get number of fuel scored in blue hub
    Logger.recordOutput("Fuel Sim/ RED SCORE",FuelSim.Hub.RED_HUB.getScore()); // get number of fuel scored in red hub
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