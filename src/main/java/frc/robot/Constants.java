package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {

    public class FIELD {
        public static final Translation2d HUB_POSE =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation2d(4.625, 4.0275) : new Translation2d(11.925, 4.0275);
        public static final Translation3d PASS_UPPER = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 6.25, 0) : new Translation3d(14.5, 6.25, 0);
        public static final Translation3d PASS_LOWER =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 1.75, 0) : new Translation3d(14.5, 1.75, 0);
    }

    public class CAN_IDS {
        public static final int CANDLE = 0;

        public static final int FLYWHEEL_MOTOR_MASTER = 45;
        public static final int FLYWHEEL_MOTOR_FOLLOWER_UPPER_RIGHT = 46;
        public static final int FLYWHEEL_MOTOR_FOLLOWER_LOWER_LEFT = 47;
        public static final int FLYWHEEL_MOTOR_FOLLOWER_LOWER_RIGHT = 48;

        public static final int RACK_MOTOR = 50;
        public static final int FEEDER_MOTOR = 51;
        public static final int INTAKE_ROLLER_MASTER = 52;
        public static final int INTAKE_FOLLOWER_ROLLER = 53;
        public static final int INDEX_MOTOR_MASTER = 54;
        public static final int INDEX_MOTOR_FOLLOWER = 55;
    }
}
