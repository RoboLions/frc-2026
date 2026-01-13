package frc.robot.lib.util;

import edu.wpi.first.math.util.Units;

public final class Conversions {

  /** Utility class, so constructor is private. */
  private Conversions() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Converts Angular Speed to Linear Speed. Note that the returned time base is not specified. This
   * is due to the conversion being time base agnostic. An angular speed input in a "per second"
   * time base will output the linear speed in a "per second" time base.
   *
   * @param w The angular speed (omega) in radians to convert to linear speed.
   * @param radius The radius at which to calculate the linear speed, in the desired units
   * @return Linear speed in the same units as the radius converted from angular speed (omega) in
   *     radians
   */
  public static double angularSpeedToLinearSpeed(double w, double radius) {
    // https://en.wikipedia.org/wiki/Angular_velocity
    return w * radius;
  }

  public static double rotationalSpeedToLinearSpeed(double w, double radius) {
    return angularSpeedToLinearSpeed(Units.rotationsToRadians(w), radius);
  }

  /**
   * Converts Linear Speed to Angular Speed. Note that the returned time base is not specified. This
   * is due to the conversion being time base agnostic. A linear speed input in a "per second" time
   * base will output the angular speed in a "per second" time base.
   *
   * @param v The linear speed to convert to angular speed (omega) in radians.
   * @param radius The radius of rotation, in the same distance units as the linear speed.
   * @return Angular speed (omega) in radians converted from linear speed
   */
  public static double linearSpeedToAngularSpeed(double v, double radius) {
    // https://en.wikipedia.org/wiki/Angular_velocity
    return v / radius;
  }

  public static double linearSpeedToRotationalSpeed(double v, double radius) {
    return Units.radiansToRotations(linearSpeedToAngularSpeed(v, radius));
  }

  /**
   * Converts an angular distance measurement to a linear distance measurment.
   *
   * @param theta The angular distance to convert to linear distance in radians.
   * @param radius The radius of rotation, in the desired linear distance units.
   * @return Linear distance in the same units as the radius of rotation, converted from angular
   *     distance
   */
  public static double radiansToDistance(double theta, double radius) {
    // https://en.wikipedia.org/wiki/Angular_displacement
    return theta * radius;
  }

  public static double rotationsToDistance(double rotations, double radius) {
    return radiansToDistance(Units.rotationsToRadians(rotations), radius);
  }
}
