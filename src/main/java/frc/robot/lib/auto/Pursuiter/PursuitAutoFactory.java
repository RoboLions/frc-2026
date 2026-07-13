package frc.robot.lib.auto.Pursuiter;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class PursuitAutoFactory {
    private final Supplier<Pose2d> poseSupplier;
    private final Consumer<ChassisSpeeds> outputConsumer;
    private final PursuitProfile pursuitProfile;

    private final Map<String, Command> eventMap = new HashMap<>();

    public PursuitAutoFactory(
        Supplier<Pose2d> poseSupplier, 
        Consumer<ChassisSpeeds> outputConsumer,
        PursuitProfile pursuitProfile) {
            this.poseSupplier = poseSupplier;
            this.outputConsumer = outputConsumer;
            this.pursuitProfile = pursuitProfile;
    }

    /** 
     * Binds a Command to an Event Marker in the Choreo path. Events in this pure pursuit implementation are
     * triggered by the current closest point on the path that the robot has traversed to being the same
     * path point as the Event Marker.
     * 
     * @param name The name of the Event Marker as set in the Choreo path. Will only detect exact matches.
     * @param command Command to bind to the Event. Is scheduled by CommandScheduler.getInstance().schedule().
     * Expect behavior accordingly to how this Command would behave so. 
     */
    public void addEvent(String name, Command command) {
        eventMap.put(name, command);
    }

    public void bindFactoryCommands(PursuitPath path) {
        eventMap.forEach((name, command) -> {
            path.bindCommand(name, command);
        });
    }

    /** Generates a drive command from a trajectory name using the default profile. */
    public Command followPath(String trajectoryName) {
        PursuitPath path = new PursuitPath(pursuitProfile, trajectoryName);
        bindFactoryCommands(path);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    /** Generates a drive command with a custom profile overriding the default. */
    public Command followPath(String trajectoryName, PursuitProfile customProfile) {
        PursuitPath path = new PursuitPath(customProfile, trajectoryName);
        bindFactoryCommands(path);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    public Command followPath(String trajectoryName, boolean flipX, boolean flipY, Translation2d centerPoint) {
        PursuitPath path = new PursuitPath(pursuitProfile, trajectoryName);
        path.flipPath(flipX, flipY, centerPoint);
        bindFactoryCommands(path);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    public Command followPath(String trajectoryName, boolean flipX, boolean flipY, Translation2d centerPoint, PursuitProfile profile) {
        PursuitPath path = new PursuitPath(profile, trajectoryName);
        path.flipPath(flipX, flipY, centerPoint);
        bindFactoryCommands(path);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    /** Triggers a registered event command by its string identifier. */
    public Command getEvent(String name) {
        return eventMap.getOrDefault(name, Commands.none());
    }
}
