package frc.lib.Pursuiter.util;

import edu.wpi.first.units.measure.AngularVelocity;

// all units in meters
public record PointConstraints(double vx, double vy, double ax, double ay, AngularVelocity omega) {
  public PointConstraints(double vx, double vy, double ax, double ay, AngularVelocity omega) {
    this.vx = vx;
    this.vy = vy;
    this.ax = ax;
    this.ay = ay;
    this.omega = omega;
  }
}
