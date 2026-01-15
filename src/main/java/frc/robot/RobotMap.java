package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.Limelight;
import frc.robot.subsystems.interfaces.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class RobotMap {

  private static SendableChooser<Command> autoChooser;

  /* state machine instances */
  // public static final DrivetrainStateMachine drivetrainStateMachine = new DrivetrainStateMachine();

  /* Xbox controllers */
  public static final XboxController manipulatorController = new XboxController(1);
  public static final XboxController driverController = new XboxController(0);

  /* Auto objects */

  public static Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public static void init() {
    Shooter.init();
    Limelight.init();
    Swerve.init();

    // drivetrainStateMachine.init();
  }

  public static void simulationCalls() {
    Swerve.teleopDrive();
    Swerve.simulationPeriodic();

    if (driverController.getXButtonPressed()) {
      Swerve.resetOdometry();
    }
  }
}
// initiate bomb sequence
