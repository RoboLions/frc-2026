# Pursuiter

Robolions 1261's pure pursuit controller for Swerve Drive, built to work directly with Choreo `.traj` files.

## What it does

Pursuiter takes a Choreo path and turns it into a `PursuitPath`, which you can convert straight into a WPILib `Command`. A pure pursuit controller drives the robot toward a moving "look-ahead" point on the path, recalculating that point as the robot moves. This makes it more resilient to disturbances mid-path than a purely time-based trajectory follower.

**Requirements:** Swerve drive only. Must also use Choreo lmao.

## Core pieces

**`PursuitAutoFactory`**
Loads and manages `.traj` files. It can:
- Bind commands to Choreo event markers with `addEvent(...)` — these trigger based on progress along the path, not proximity, so overlapping paths won't cause double-triggers. Each registered command is matched against every loaded `.traj` file; if a command isn't found in a given path, a warning is thrown. These warnings are safe to ignore if that's intentional (e.g. the command only applies to some paths). This may be disabled when adding the command to `PursuitAutoFactory`.
- Mirror a path across the X and/or Y axis about a provided center point via `followPath(...)`.
- Log the active trajectory for visualization (toggle via `PursuitProfile`).
- Take a pose supplier, an odometry-reset function, and a `PursuitProfile`.
- Register autos into a built-in `SendableChooser`.

**`PursuitProfile`**
Bundles the tuning values a path needs:
- Endpoint tolerance (position and rotation)
- Look-ahead distance
- A PID controller for translational correction mid-path
- A separate PID controller just for the endpoint, since profiled speeds slow down near the end of a path and can make it hard to settle within tolerance. That combined with the lack of a new Look-ahead point can cause weird behavior at times.
- A logging toggle for the default logging method.

## Quick start

**1. Define a profile and factory**

```java
private static PursuitProfile profile =
    new PursuitProfile(
        Units.Meters.of(0.1),          // endpoint position tolerance
        Units.Degrees.of(5),           // endpoint rotation tolerance
        Units.Meters.of(0.3),          // look-ahead distance
        new PIDController(1.0, 0, 0),  // mid-path translation PID
        new PIDController(2.0, 0, 0),  // endpoint translation PID
        new PIDController(2.0, 0, 0),  // rotation PID
        true);                         // enable default logging

private static PursuitAutoFactory autoFactory =
    new PursuitAutoFactory(Swerve::getPose, Swerve::setFieldChassisSpeeds, profile);
```

**2. Bind event-marker commands and register autos**

```java
public static void init() {
  autoFactory.addEvent("INTAKE", Commands.runOnce(() -> System.out.println("Intake!")));
  autoFactory.addEvent("INTAKE_STOP", Commands.runOnce(() -> System.out.println("Intake stopped")));
  autoFactory.addEvent("REV_SHOT", Commands.runOnce(() -> System.out.println("Revving shot")));

  autoFactory.registerAutoCommand("Test Auto!", testCommand());
  autoFactory.registerAutoCommand("nothing", new Command() {});
}
```

**3. Schedule the selected auto**

```java
@Override
public void autonomousInit() {
  CommandScheduler.getInstance().schedule(autoFactory.getSelectedAuto());
}
```

**4. Chain paths together**

```java
private static Command testCommand() {
  return autoFactory
      .followPath("L1.traj", false, true, FieldMap.center)
      .andThen(autoFactory.followPath("L2.traj", false, true, FieldMap.center))
      .andThen(autoFactory.followPath("L3.traj", false, true, FieldMap.center));
}
```

OR

```java
  private static Command testCommand2(Supplier<Pose2d> odometrySupplier, Consumer<ChassisSpeeds> setChassisSpeeds) {
    PursuitPath l1 = new PursuitPath(profile, "L1.traj");
    PursuitPath l2 = new PursuitPath(profile, "L2.traj");
    PursuitPath l3 = new PursuitPath(profile, "L3.traj");
    l1.append(l2);
    l1.append(l3);

    return l1.toCommand(odometrySupplier, setChassisSpeeds);
  }
```

## Full example

```java
package frc.robot.lib.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.auto.Pursuiter.PursuitAutoFactory;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;
import frc.robot.lib.auto.Pursuiter.helpers.FieldMap;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class AutoSubsystem {

  private static PursuitProfile profile =
      new PursuitProfile(
          Units.Meters.of(0.1),
          Units.Degrees.of(5),
          Units.Meters.of(0.3),
          new PIDController(1.0, 0, 0),
          new PIDController(2.0, 0, 0),
          new PIDController(2.0, 0, 0),
          true);

  private static PursuitAutoFactory autoFactory =
      new PursuitAutoFactory(Swerve::getPose, Swerve::setFieldChassisSpeeds, profile);

  public static void init() {
    autoFactory.addEvent("INTAKE", Commands.runOnce(() -> System.out.println("Intake!")));
    autoFactory.addEvent("INTAKE_STOP", Commands.runOnce(() -> System.out.println("Intake stopped")));
    autoFactory.addEvent("REV_SHOT", Commands.runOnce(() -> System.out.println("Revving shot")));

    autoFactory.registerAutoCommand("test 1", testCommand());
    autoFactory.registerAutoCommand("test 2", testCommand2(Swerve::getPose, Swerve::setFieldChassisSpeeds));
    autoFactory.registerAutoCommand("nothing", new Command() {});
  }

  public static void autoInit() {
    CommandScheduler.getInstance().schedule(autoFactory.getSelectedAuto());
  }

  private static Command testCommand() {
    return autoFactory
        .followPath("L1.traj", false, true, FieldMap.center)
        .andThen(autoFactory.followPath("L2.traj", false, true, FieldMap.center))
        .andThen(autoFactory.followPath("L3.traj", false, true, FieldMap.center));
  }

  private static Command testCommand2(Supplier<Pose2d> odometrySupplier, Consumer<ChassisSpeeds> setChassisSpeeds) {
    PursuitPath l1 = new PursuitPath(profile, "L1.traj");
    PursuitPath l2 = new PursuitPath(profile, "L2.traj");
    PursuitPath l3 = new PursuitPath(profile, "L3.traj");
    l1.append(l2);
    l1.append(l3);

    return l1.toCommand(odometrySupplier, setChassisSpeeds);
  }
}
```
