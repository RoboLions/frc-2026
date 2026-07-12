package frc.robot.lib.auto.Pursuiter;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;
import frc.robot.lib.auto.Pursuiter.helpers.PoseTolerance;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;

public class PursuitAuto {
    PoseTolerance poseTolerance;
    Distance lookAhead;
    List<PathPoint> pathPoints;
    PIDController xController = new PIDController(0.5, 0, 0);
    PIDController yController = new PIDController(0.5, 0, 0);
    PIDController headingController = new PIDController(1.0, 0, 0);
    
    /**
     * 
     * @param metersTolerance
     * @param degreesTolerance
     * @param lookAheadDistance
     * @param trajectoryName Should include the .traj suffix. Ex: "leave.traj" for a Choreo path called leave.
    */
    public PursuitAuto(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
    }

    public PursuitAuto(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        PIDController xController,
        PIDController yController,
        PIDController headingController,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.xController = xController;
        this.yController = yController;
        this.headingController = headingController;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
    }
}
