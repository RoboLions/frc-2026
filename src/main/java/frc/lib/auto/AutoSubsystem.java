package frc.lib.auto;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Pursuiter.PursuitAutoFactory;
import frc.lib.Pursuiter.PursuitPath;
import frc.lib.Pursuiter.PursuitProfile;
import frc.lib.Pursuiter.helpers.FieldMap;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class AutoSubsystem {

  private static PursuitProfile profile =
      new PursuitProfile(
          Units.Meters.of(0.1),
          Units.Degrees.of(5.0),
          Units.Meters.of(0.3),
          new PIDController(1.0, 0, 0),
          new PIDController(3.0, 0, 0),
          new PIDController(7.5, 0, 0),
          true);
  private static PursuitAutoFactory autoFactory =
      new PursuitAutoFactory(Swerve::getPose, Swerve::setFieldChassisSpeeds, profile);

  public static void init() {
    autoFactory.addEvent("INTAKE", Commands.runOnce(() -> System.out.println("Intake!")));
    autoFactory.addEvent("INTAKE_STOP", Commands.runOnce(() -> System.out.println("INTAKE STOPPP!!!!!!!!!")));
    autoFactory.addEvent("REV_SHOT", Commands.runOnce(() -> System.out.println("REV SHOT!!!!!!!!!")));

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
