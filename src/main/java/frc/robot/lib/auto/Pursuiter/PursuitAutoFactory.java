package frc.robot.lib.auto.Pursuiter;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

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

    public PursuitAutoFactory(
        Supplier<Pose2d> poseSupplier, 
        Consumer<ChassisSpeeds> outputConsumer,
        PursuitProfile pursuitProfile,
        Subsystem driveSubsystem) {
            this.poseSupplier = poseSupplier;
            this.outputConsumer = outputConsumer;
            this.pursuitProfile = pursuitProfile;
    }

    public void registerCommand(String name, Command command) {
        eventMap.put(name, command);
    }

    /** Generates a drive command from a trajectory name using the default profile. */
    public Command followPath(String trajectoryName) {
        PursuitPath path = new PursuitPath(pursuitProfile, trajectoryName);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    /** Generates a drive command with a custom profile overriding the default. */
    public Command followPath(String trajectoryName, PursuitProfile customProfile) {
        PursuitPath path = new PursuitPath(customProfile, trajectoryName);
        return path.toCommand(poseSupplier, outputConsumer);
    }

    /** Triggers a registered event command by its string identifier. */
    public Command triggerEvent(String name) {
        return eventMap.getOrDefault(name, Commands.none());
    }
}
