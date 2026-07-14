package frc.robot.lib.auto.Pursuiter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;

public final record PursuitProfile(
    Distance metersTolerance,
    double degreesTolerance,
    Distance lookAheadDistance,
    PIDController translationController,
    PIDController endPointController,
    PIDController headingController,
    boolean logToggle) {}
