package frc.robot.lib.auto.Pursuiter.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class PursuitEventMarker {
  private String name;
  private Command command;
  private boolean hasTriggered;

  public PursuitEventMarker(String name, Command command) {
    this.command = command;
    this.name = name;
  }

  public void bindCommand(Command command) {
    this.command = command;
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
    if (command != null && !hasTriggered) {
      CommandScheduler.getInstance().schedule(command);
      hasTriggered = true;
      return true;
    }
    return false;
  }

  public void reset() {
    this.hasTriggered = false;
  }
}
