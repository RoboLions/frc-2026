package frc.robot.lib.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.Hood;
import frc.robot.subsystems.swerve.Swerve;

public class AutoCommnads {

    public static Trigger exampleTrigger() {
        return new Trigger(() -> false);
    }

    public static Command exampleCommand() {
        return Commands.runOnce(() -> System.out.println("Hello World!"));
    }
    
    public static Command SwerveStop() {
        return Commands.runOnce(() -> Swerve.zeroCommand());
    }

    public static Command PrintItem(String string) {
        return Commands.runOnce(() -> System.out.println(string));
    }

    public static Command setTurretTrack() {
        return Commands.run(() -> Hood.simulateTurretAngle(Swerve.getPose(), 
                                                           Constants.Hood.HUB_POSE, 
                                                           Swerve.getYawAsRadians(), 
                                                           Swerve.getYawRateAsRad()));
    }
}
