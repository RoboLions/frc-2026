package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public class Constants {

    public class Hood {
        public static final int FRONT_PIVOT_MOTOR = 0;
        public static final int BACK_PIVOT_MOTOR = 0;

        public static final double G = 9.807;
        public static final double THETA_ANGLE_FROM_SHOOTER = 90; //this is measured angled which the ball exits the shooter
    }

    public class Shooter {
        public static final int FRONT_MASTER_FLYWHEEL_MOTOR = 0;
        public static final int FRONT_FOLLOWER_FLYWHEEL_MOTOR = 0;
        public static final int BACK_FLYWHEEL_MOTOR = 0;
    }

    public class LimeLight {
        public static final Pose3d known_pose_blue_left =
            new Pose3d(new Translation3d(1.252857, 5.547879, 0.0), new Rotation3d(0, 0, Math.PI));
      
        public static final Map<String, Double> last_timestamps = new HashMap<String, Double>();
    }
    
}
