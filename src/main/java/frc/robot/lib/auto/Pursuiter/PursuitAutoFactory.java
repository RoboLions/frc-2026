package frc.robot.lib.auto.Pursuiter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PursuitAutoFactory {
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<ChassisSpeeds> outputConsumer;
  private final PursuitProfile pursuitProfile;

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private final Map<String, Command> eventMap = new HashMap<>();
  private PursuitPath currentPath;
  private boolean pathInterrupted; 

  public PursuitAutoFactory(
      Supplier<Pose2d> poseSupplier,
      Consumer<ChassisSpeeds> outputConsumer,
      PursuitProfile pursuitProfile) {
    this.poseSupplier = poseSupplier;
    this.outputConsumer = outputConsumer;
    this.pursuitProfile = pursuitProfile;
  }

  /**
   * Binds a Command to an Event Marker in the Choreo path. Events in this pure pursuit
   * implementation are triggered by the current closest point on the path that the robot has
   * traversed to being the same path point as the Event Marker.
   *
   * @param name The name of the Event Marker as set in the Choreo path. Will only detect exact
   *     matches.
   * @param command Command to bind to the Event. Is scheduled by
   *     CommandScheduler.getInstance().schedule(). Expect behavior accordingly to how this Command
   *     would behave so.
   */
  public void addEvent(String name, Command command) {
    eventMap.put(name, command);
  }

  public void bindFactoryCommands(PursuitPath path) {
    eventMap.forEach(
        (name, command) -> {
          path.bindCommand(name, command);
        });
  }

  private Command wrapWithKillSwitch(Command command) {
    return command
      .beforeStarting(() -> pathInterrupted = false)
      .until(() -> pathInterrupted)
      .finallyDo(interrupted -> {
        if (pathInterrupted) {
          outputConsumer.accept(new ChassisSpeeds());
        }
      });
  }

  /** Generates a drive command from a trajectory name using the default profile. */
  public Command followPath(String trajectoryName) {
    currentPath = new PursuitPath(pursuitProfile, trajectoryName);
    bindFactoryCommands(currentPath);
    return wrapWithKillSwitch(currentPath.toCommand(poseSupplier, outputConsumer));
  }

  public Command followPath(String trajectoryName, PursuitProfile customProfile) {
    currentPath = new PursuitPath(customProfile, trajectoryName);
    bindFactoryCommands(currentPath);
    return wrapWithKillSwitch(currentPath.toCommand(poseSupplier, outputConsumer));
  }

  public Command followPath(
      String trajectoryName, boolean flipX, boolean flipY, Translation2d centerPoint) {
    currentPath = new PursuitPath(pursuitProfile, trajectoryName);
    currentPath.flipPath(flipX, flipY, centerPoint);
    bindFactoryCommands(currentPath);
    return wrapWithKillSwitch(currentPath.toCommand(poseSupplier, outputConsumer));
  }

  public Command followPath(
      String trajectoryName,
      boolean flipX,
      boolean flipY,
      Translation2d centerPoint,
      PursuitProfile profile) {
    currentPath = new PursuitPath(profile, trajectoryName);
    currentPath.flipPath(flipX, flipY, centerPoint);
    bindFactoryCommands(currentPath);
    return wrapWithKillSwitch(currentPath.toCommand(poseSupplier, outputConsumer));
  }

  /** Triggers a registered event command by its string identifier. */
  public Command getEvent(String name) {
    return eventMap.getOrDefault(name, Commands.none());
  }

  public void killCurrentPath() {
    this.pathInterrupted = true;
  }

  public SendableChooser<Command> getAutoChooser() {
    return autoChooser;
  }

  public Command getSelectedAuto() {
    return autoChooser.getSelected();
  }

  public void registerAutoCommand(String routineName, Command auto) {
    autoChooser.addOption(routineName, auto);
    SmartDashboard.putData(autoChooser);
  }
}
