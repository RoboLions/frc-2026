package frc.lib.Pursuiter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public final record PursuitProfile(
    Distance metersTolerance,
    Angle degreesTolerance,
    Distance lookAheadDistance,
    PIDController translationController,
    PIDController endPointController,
    PIDController headingController,
    boolean logToggle) {}
