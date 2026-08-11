package frc.robot.subsystems.interfaces;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.swerve.Swerve;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;

public class Sotm {
    
    public static Translation2d getRobotFieldVelocity() {
    return new Translation2d(Swerve.getFieldSpeeds().vxMetersPerSecond, Swerve.getFieldSpeeds().vyMetersPerSecond);
    }   

    private static final ArrayList<TimeOfFlightPoint> TIME_OF_FLIGHT_LOOKUP_TABLE =
    new ArrayList<>();

private static class TimeOfFlightPoint {
    double currentDistance;
    double timeSeconds;

    TimeOfFlightPoint(double currentDistance, double timeSeconds) {
        this.currentDistance = currentDistance;
        this.timeSeconds = timeSeconds;
    }
}

static {
    // Starting estimates replace when get the robot
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(1.70, 0.8939));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(2.09, 1.0096));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(2.51, 1.0348));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(3.00, 1.0686));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(3.50, 1.0829));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(3.81, 1.1021));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(4.20, 1.1094));
    TIME_OF_FLIGHT_LOOKUP_TABLE.add(new TimeOfFlightPoint(4.50, 1.1202));
}

public static double getTimeOfFlight(double currentDistance) {
    if (currentDistance <= TIME_OF_FLIGHT_LOOKUP_TABLE.get(0).currentDistance) {
        return TIME_OF_FLIGHT_LOOKUP_TABLE.get(0).timeSeconds;
    }

    for (int i = 0; i < TIME_OF_FLIGHT_LOOKUP_TABLE.size() - 1; i++) {
        TimeOfFlightPoint first = TIME_OF_FLIGHT_LOOKUP_TABLE.get(i);
        TimeOfFlightPoint second = TIME_OF_FLIGHT_LOOKUP_TABLE.get(i + 1);

        if (currentDistance <= second.currentDistance) {
            double fraction = (currentDistance - first.currentDistance)
                / (second.currentDistance - first.currentDistance);

            return first.timeSeconds
                + fraction * (second.timeSeconds - first.timeSeconds);
        }
    }

    return TIME_OF_FLIGHT_LOOKUP_TABLE
        .get(TIME_OF_FLIGHT_LOOKUP_TABLE.size() - 1).timeSeconds;
}
   

    public static Translation2d getLeadTarget(Translation2d target){
    Translation2d leadTarget = target.minus(getRobotFieldVelocity() .times(getTimeOfFlight(Swerve.getPose().getTranslation().getDistance(Constants.FIELD.HUB_POSE))));
    return leadTarget;
    }
    // leadTarget = target - robotFieldVelocity * timeOfFlight;

    
}
