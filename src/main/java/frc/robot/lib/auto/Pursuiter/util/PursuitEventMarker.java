package frc.robot.lib.auto.Pursuiter.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PursuitEventMarker {
    private String name;
    private Command command;
    private boolean hasTriggered;
    private BooleanSupplier condition;

    public PursuitEventMarker (Command command, BooleanSupplier condition) {
        this.command = command;
        this.condition = (condition != null) ? condition : () -> true;    
    }

    public PursuitEventMarker (String name, Command command) {
        this(command, () -> true);    
        this.name = name;
    }

    public void bindCommand(Command command) {
        this.command = command;
    }

    public void setCondition(BooleanSupplier supplier) {
        this.condition = supplier;
    }

    public Command getCommand() {
        return command;
    }

    public String getName() {
        return name;
    }

    public boolean hasTriggered() {
        return hasTriggered;
    }
    
    public boolean trigger() {
        if (command != null && !hasTriggered && condition.getAsBoolean()) {
            CommandScheduler.getInstance().schedule(command);
            hasTriggered = true;
            return true;
        } return false;
    }

    public void reset() {
        this.hasTriggered = false;
    }
}
