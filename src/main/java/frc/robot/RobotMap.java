package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.auto.AutoSubsystem;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.interfaces.Hood;
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

    if (DriverStation.isTeleopEnabled()) {
      Swerve.simulationDrive();
      scoringStateMachine.setNextState();
    }
  }

  public static void simulationAuto() {
    autoSubsystem.scheduleAutoSimulation();
  }

  public static void simulateFuelInit() {

  }

  public static void simulateFuelPeriodics() {
    if (Hood.SimulationObjects.isSimulationShooting) {
      Hood.launchFuel();
    }

    // FuelSim.getInstance().clearFuel(); // clears all fuel from the field

    Logger.recordOutput("BLUE SCORE", FuelSim.Hub.BLUE_HUB.getScore()); // get number of fuel scored in blue hub
    // FuelSim.Hub.RED_HUB.getScore(); // get number of fuel scored in red hub
    // FuelSim.Hub.[BLUE/RED]_HUB.resetScore(); // resets the score of the blue/red hub
  }
}
// initiate bomb sequence
