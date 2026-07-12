package frc.robot.lib.auto.Pursuiter;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.auto.Pursuiter.helpers.PoseTolerance;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;

public class PursuitAuto {

    PoseTolerance poseTolerance;
    Distance lookAhead;
    List<PathPoint> pathPoints;

    public PursuitAuto(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
    }

}
