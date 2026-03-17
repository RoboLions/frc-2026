package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {

    public class Hood {
        public static final double G = 9.80665;
        public static final double THETA_ANGLE_FROM_SHOOTER = 102; //this is measured angled which the ball exits the shooter

        public static final double MAX_HOOD_ANGLE_DEG = 42.0;
        public static final double BASE_HOOD_ANGLE_DEG = 18.75;
        public static final double DEGREE_RATIO = 30.1;

        public static final double YAW_COMPENSATION_LATENCY_MS = 100.0;

        public static final double HEIGHT_FROM_BOT_TO_TARGET = 1.117; //meters
        public static final Translation2d TURRET_ROBOT_OFFSET = new Translation2d(0.14605, -0.14605); //meters

        public static final Translation2d HUB_POSE =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation2d(4.625, 4.025) : new Translation2d(11.91642, 4.03814);
        public static final Translation3d PASS_UPPER = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 6.75, 0) : new Translation3d(14.5, 6.75, 0);
        public static final Translation3d PASS_LOWER =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 1.25, 0) : new Translation3d(14.5, 1.25, 0);
    }

    public class Shooter {
        public static final double POWER_GAIN_MULTIPLIER = 2.025;
    }
    
    public class CAN_IDS {
        public static final int HOOD_PIVOT_MOTOR = 58;
        public static final int FLYWHEEL_MOTOR_RIGHT = 57;
        public static final int FLYWHEEL_MOTOR_LEFT = 56;
        public static final int TURRET_AZIMUTH_MOTOR = 54;

        public static final int INDEX_MOTOR = 50;
        public static final int FEED_MOTOR = 52;
        public static final int INTAKE_ROLLER = 53;
        public static final int INTAKE_FOLLOWER_ROLLER = 59;
        public static final int RACK_MOTOR = 51;
    }
}
