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
    autoFactory.addEvent("INTAKE_STOP", Commands.runOnce(() -> System.out.println("INTAKE STOPPP!!!!!!!!!")));
    autoFactory.addEvent("REV_SHOT", Commands.runOnce(() -> System.out.println("REV SHOT!!!!!!!!!")));

    autoFactory.registerAutoCommand("test 1", testCommand());
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
}
