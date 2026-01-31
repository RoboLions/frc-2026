package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    public class Hood {
        public static final double G = 9.80665;
        public static final double THETA_ANGLE_FROM_SHOOTER = 90; //this is measured angled which the ball exits the shooter
        public static final double TIME_OF_FLIGHT_SCALE = 1.0;

        public static final Translation2d HUB_POSE =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation2d(4.62744, 4.03648) : new Translation2d(11.91642, 4.03814);

        public static final Translation3d PASS_DEPOT = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 7, 0) : new Translation3d(15, 7, 0);
        public static final Translation3d PASS_OUTPOST =  DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? new Translation3d(1.5, 1, 0) : new Translation3d(15, 1, 0);

        public static final double HEIGHT_FROM_BOT_TO_TARGET = 1.25;
    }

    public class LimeLight {
        public static final Pose3d known_pose_blue_left =
            new Pose3d(new Translation3d(1.252857, 5.547879, 0.0), new Rotation3d(0, 0, Math.PI));
    }
    
    public class CAN_IDS {
        public static final int HOOD_PIVOT_MOTOR = 0;
        public static final int FLYWHEEL_MOTOR_MASTER = 0;
        public static final int FLYWHEEL_MOTOR_FOLLOWER = 0;
        public static final int TURRET_AZIMUTH_MOTOR = 0;

        public static final int FRONT_PIVOT_MOTOR = 0;
        public static final int BACK_PIVOT_MOTOR = 0;

        public static final int MASTER_INTAKE_MOTOR = 0;
        public static final int FOLLOWER_INTAKE_MOTOR = 0;
        public static final int INDEX_MOTOR = 0;
        public static final int FEED_MOTOR = 0;
        public static final int INT_PIVOT_MOTOR = 0;

        public static final int LEFT_CLIMB_MOTOR = 0;
        public static final int RIGHT_CLIMB_MOTOR = 0;


    }
}
