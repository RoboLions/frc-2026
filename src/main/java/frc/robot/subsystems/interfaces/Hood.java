package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.swerve.Swerve;

public class Hood {
  private static final TalonFX mFollowerPivotMotor =
    new TalonFX(Constants.CAN_IDS.BACK_FLYWHEEL_MOTOR);
  private static final TalonFX mMasterPivotMotor =
    new TalonFX(Constants.CAN_IDS.FRONT_MASTER_FLYWHEEL_MOTOR);  
  public static final StatusSignal<Angle> mBackMotorVelo = mFollowerPivotMotor.getPosition();
  public static final StatusSignal<Angle> mFrontMotorVelo = mMasterPivotMotor.getPosition();
  
  public class SimulationObjects {
    public static double desiredTurretAngleRobotRelRad;
    public static double desiredHoodAngleRobotRel;
    public static double totalShotVelocity;
    public static double literalShotHoodRad;

    private static Timer timer = new Timer();
  }
  
  public static void init() {
      TalonFXConfiguration frontPivotConfig = new TalonFXConfiguration();
      frontPivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
      frontPivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
      frontPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      frontPivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
      frontPivotConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
      frontPivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
      frontPivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;
      frontPivotConfig.Slot0.kS = 0;
      frontPivotConfig.Slot0.kV = 0;
      frontPivotConfig.Slot0.kA = 0;
      frontPivotConfig.Slot0.kP = 0;
      frontPivotConfig.Slot0.kI = 0;
      frontPivotConfig.Slot0.kD = 0;
      frontPivotConfig.MotionMagic.MotionMagicAcceleration = 1;
      frontPivotConfig.Feedback.SensorToMechanismRatio = 1 / 1;
      frontPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      frontPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      frontPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      frontPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      frontPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
      frontPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1;
      frontPivotConfig.ClosedLoopGeneral.ContinuousWrap = false;
      mMasterPivotMotor.getConfigurator().apply(frontPivotConfig);
      mFollowerPivotMotor.getConfigurator().apply(frontPivotConfig);
  }

  /** 
   * The fully abstracted method that tracks the turret, no nonsense and all setup already.
   * Call to update the turret position.
   */
  public static void turretTrack() {
    Hood.simulateTurretAngle(Swerve.getPose(), 
                             Constants.Hood.HUB_POSE, 
                             Swerve.getYawAsRadians(), 
                             Swerve.getYawRateAsRad(),
                             Swerve.getFieldSpeeds(),
                             Swerve.getChassisAcceleration(),
                             Swerve.getLoopLatencySec());
  }

  /**
   * Calculates the required launch velocity (m/s) for a projectile to reach a target 
   * at a given distance and height, accounting for gravity and robot motion.
   * * <p>The calculation uses a base minimum velocity to clear the physical 
   * constraints of the target, then adds an exponential decay offset based 
   * on distance and the current magnitude of the robot's velocity vector.</p>
   *
   * @param d  The horizontal distance to the target (meters/inches).
   * @param h  The vertical height of the target relative to the shooter (meters/inches).
   * @param vx The current velocity of the robot along the X-axis.
   * @param vy The current velocity of the robot along the Y-axis.
   * @return   The calculated launch velocity magnitude.
   */
  private static double sampleVelocity(double d, double h, double vx, double vy) {
    double velocity = Math.sqrt(
      Constants.Hood.G * (Math.sqrt(d * d + h * h) + h)) // the minimum line. go below this velocity and we will hit the SIDE.
        + 5 * Math.pow(Math.E, -(0.75 * d)); // the offset line, adjust as desired

    Logger.recordOutput("Turret Sim/ Velocity Boost Factor", 5 * Math.pow(Math.E, -2 * d));
        
    return velocity;
  }

  /**Calculates the required launch angle for a 2D projectile to hit a target.
  * Uses the standard trajectory equation: 
  * h = d*tan(θ) - (g*d²) / (2*v²*cos²(θ))
  * 
  * @param v The velocity as the piece exits the robot's shooter.
  * @param d The distance from the exit point to the target.
  * @param h The height from the exit point to the target.
  * @return The angle in degrees relative to the horizontal. Returns Double.NaN 
   * if the target is out of range at the current velocity.
   * * @note This implementation calculates the "High Arc" (Lob) solution. 
   * The physical hood offset is applied before returning.
  */
  public static double calculateHoodAngle(double v, double d, double h) {
    final double g = Constants.Hood.G;
    double v2 = v * v;
    double v4 = v * v * v * v;
    double d2 = d * d;
    
        double discriminant = v4 - g * (g * d2 + 2.0 * h * v2);
        if (discriminant < 0.0 || d == 0.0) {
            return Double.NaN; //No physical solution
        }
      double numerator = v2 + Math.sqrt(discriminant);
      double denominator = g * d;
      double angleRadians = Math.atan(numerator / denominator);
      double o = Math.toDegrees(angleRadians);
    return Constants.Hood.THETA_ANGLE_FROM_SHOOTER - o;
  } 

  /** Is the mirror double to the algorithm above but just in launch, not a hood.
  * 
  * @param v The velocity as the piece exits the robot's shooter.
  * @param d The distance from the exit point to the target.
  * @param h The height from the exit point to the target.
  * @return angle in Rad that the particle will launch from relative to the horizontal plane.
  */
  public static double calculateLaunchAngleRad(double v, double d, double h) {
    final double g = Constants.Hood.G;
    double v2 = v * v;
    double v4 = v * v * v * v;
    double d2 = d * d;
    
        double discriminant = v4 - g * (g * d2 + 2.0 * h * v2);
        if (discriminant < 0.0 || d == 0.0) {
            return Double.NaN; //No physical solution
        }
      double numerator = v2 + Math.sqrt(discriminant);
      double denominator = g * d;
      double angleRadians = Math.atan(numerator / denominator);
    return angleRadians;
  } 

  /** Offsets the target position along the line from the robot to the target.
  *
  * This is used to intentionally aim *behind* the hub's center rather than directly at
  * its center. By shifting the target point backward along the robot-to-hub
  * bearing, the shot contacts the back of the hub at a slight angle instead of
  * squaring up in the middle. This promotes more consistent backspin interaction
  * with the hub and improves shot forgiveness and accuracy.
  *
  * A positive transformation distance moves the target farther away from the
  * robot (past the hub), while a negative value pulls the aim point closer.
  *
  * @param currPose Current robot pose on the field
  * @param targetPose Original target position (hub center)
  * @param transformationInMeters Distance to shift the target along the bearing
  * @return Transformed target translation used for aiming
  */
  public static Translation2d transformTarget(Pose2d currPose, Translation2d targetPose, double transformationInMeters) {
    Translation2d deltaTranslation = targetPose.minus(currPose.getTranslation());
    double radAngle = Math.atan2(deltaTranslation.getY(), deltaTranslation.getX());
    return new Translation2d(targetPose.getX() + transformationInMeters * Math.cos(radAngle), targetPose.getY() + transformationInMeters * Math.sin(radAngle));
  }

  /**
   * Simulates and calculates the required turret and hood kinematics to hit a target while the robot is in motion.
   * * <p>This method uses a "Fake Target" (Imaginary Target) approach to compensate for:
   * <ul>
   * <li><b>Robot Velocity:</b> Offsets the aim to cancel out the lateral momentum of the launched projectile.</li>
   * <li><b>System Latency:</b> Accounts for the delay in the control loop using field acceleration.</li>
   * <li><b>Time of Flight (TOF):</b> Predicts the projectile's travel time to adjust for displacement.</li>
   * </ul>
   *
   * @param currPose           The current 2D field pose of the robot (meters).
   * @param targetPose         The static 2D field position of the goal/hub (meters).
   * @param robotFieldYaw      The current rotation of the robot relative to the field (radians).
   * @param robotYawRate       The angular velocity of the robot (rad/s).
   * @param fieldRobotSpeeds   The current velocity vector of the chassis in field-relative coordinates (m/s).
   * @param fieldRobotAccel    The current acceleration vector of the chassis in field-relative coordinates (m/s²).
   * @param loopLatencySec     The time delay of the control loop (seconds) for preemptive compensation.
   * * @implNote 
   * <b>Frame of Reference:</b>
   * <ul>
   * <li>The "Fake Target" is calculated in the <b>Field Coordinate Frame</b>.</li>
   * <li>The final Turret Angle is converted to the <b>Robot Coordinate Frame</b> (Robot-Relative).</li>
   * <li>The Hood Angle is relative to the <b>Horizontal Plane</b> (0° = Floor Parallel).</li>
   * </ul>
   */
  public static void simulateTurretAngle(Pose2d currPose, 
                                         Translation2d targetPose, 
                                         double robotFieldYaw,
                                         double robotYawRate,
                                         ChassisSpeeds fieldRobotSpeeds,
                                         ChassisSpeeds fieldRobotAccel,
                                         double loopLatencySec) 
  {
    Translation2d transformedTarget = transformTarget(currPose, targetPose, 0.25);
    double xSpeeds = fieldRobotSpeeds.vxMetersPerSecond;
    double ySpeeds = fieldRobotSpeeds.vyMetersPerSecond;
    double xAccel = fieldRobotAccel.vxMetersPerSecond;
    double yAccel = fieldRobotAccel.vyMetersPerSecond;

    double dx = transformedTarget.getX() - currPose.getX();
    double dy = transformedTarget.getY() - currPose.getY();
    double r2 = dx * dx + dy * dy;
    double r = Math.sqrt(r2);

    double preliminaryV = sampleVelocity(r, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, xSpeeds, ySpeeds);
    double preliminaryTheta = calculateLaunchAngleRad(preliminaryV, r, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET);
    double timeOFlight = Math.sqrt(r2) / (preliminaryV * Math.cos(preliminaryTheta));

    Logger.recordOutput("Turret Sim/ Pre-emptive Distance to Goal", r);
    Logger.recordOutput("Turret Sim/ Pre-emptive Estimated Velocity", preliminaryV);
    Logger.recordOutput("Turret Sim/ Pre-emptive Estimated Launch Angle", Math.toDegrees(preliminaryTheta));
    Logger.recordOutput("Turret Sim/ Pre-emptive Estimated TOF", timeOFlight);

    Translation2d targetOffset = new Translation2d(xSpeeds, ySpeeds).times(timeOFlight);
    targetOffset = targetOffset.plus(new Translation2d(xAccel, yAccel).times(loopLatencySec));
    Translation2d imaginaryTarget = transformedTarget.minus(targetOffset);
    Pose3d FAKEPOSE = new Pose3d(new Translation3d(imaginaryTarget).plus(new Translation3d(0, 0, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET)), new Rotation3d());
    
    Logger.recordOutput("Turret Sim/ Fake Target", FAKEPOSE);

    double newDX = FAKEPOSE.getX() - currPose.getX();
    double newDY = FAKEPOSE.getY() - currPose.getY();
    double newR2 = newDX * newDX + newDY * newDY;
    double newR = Math.sqrt(newR2);

    SimulationObjects.desiredTurretAngleRobotRelRad = Math.atan2(newDY, newDX) - robotFieldYaw;
    SimulationObjects.totalShotVelocity= sampleVelocity(newR, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, xSpeeds, ySpeeds);
    SimulationObjects.desiredHoodAngleRobotRel = calculateHoodAngle(SimulationObjects.totalShotVelocity, newR, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET);
    SimulationObjects.literalShotHoodRad = calculateLaunchAngleRad(SimulationObjects.totalShotVelocity, newR, Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET);

    Logger.recordOutput("Turret Sim/ Turret 3D Pose", new Pose3d(0, 0, 0, new Rotation3d(0 , 0, SimulationObjects.desiredTurretAngleRobotRelRad)));
    Logger.recordOutput("Turret Sim/ Shot Velocity", preliminaryV);
    Logger.recordOutput("Turret Sim/ Hood Angle", SimulationObjects.desiredHoodAngleRobotRel);
    Logger.recordOutput("Turret Sim/ Shot-Angle (Adjusted)", Math.toDegrees(SimulationObjects.literalShotHoodRad));
    Logger.recordOutput("Turret Sim/ Turret Angle Field Relative", SimulationObjects.desiredTurretAngleRobotRelRad + Swerve.getPose().getRotation().getDegrees());
  }

  public static void launchFuel() {
    SimulationObjects.timer.start();

    if (!SimulationObjects.timer.hasElapsed(0.5)) {
      return;
    }

    Pose3d robot = Swerve.getPose3d();
    Translation3d initialPosition = robot.getTranslation().plus(new Translation3d(0, 0, 0.3));
    FuelSim.getInstance().spawnFuel(initialPosition, launchVectorSim().plus(
      new Translation3d(Swerve.getFieldSpeeds().vxMetersPerSecond, Swerve.getFieldSpeeds().vyMetersPerSecond, 0)));

    SimulationObjects.timer.reset();
  }

  private static Translation3d launchVectorSim() {
    double hoodAngleRad = SimulationObjects.literalShotHoodRad;
    double turretThetaRad = SimulationObjects.desiredTurretAngleRobotRelRad + Swerve.getYawAsRadians(); // make this field relative again

    double z = SimulationObjects.totalShotVelocity * Math.sin(hoodAngleRad);
    double x = SimulationObjects.totalShotVelocity * Math.cos (hoodAngleRad) * Math.cos(turretThetaRad);
    double y = SimulationObjects.totalShotVelocity * Math.cos (hoodAngleRad) * Math.sin(turretThetaRad);

    Translation3d shotVec = new Translation3d(x, y, z);

    Logger.recordOutput("Turret Sim/ Shot Vector", shotVec);
    return shotVec;
  }
}