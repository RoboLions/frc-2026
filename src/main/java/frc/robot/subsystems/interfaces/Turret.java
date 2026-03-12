package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.util.FuelSim;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class Turret {
  private static final TalonFX mHoodPivotMotor =
    new TalonFX(Constants.CAN_IDS.HOOD_PIVOT_MOTOR, "CANexternal");
  private static final TalonFX mAzimuthTurretMotor = 
    new TalonFX(Constants.CAN_IDS.TURRET_AZIMUTH_MOTOR, "CANexternal");

  private class TurretConstants {
    private static final double maxPositiveTurnAngle = Math.PI;
    private static final double maxNegaitveTurnAngle = -Math.PI;
    private static final double azimuthRotationstoRadians = (Math.PI / 20.5);
  }
  
  public class SimulationObjects {
    public static double desiredTurretAngleRobotRelRad;
    public static double desiredHoodAngleRobotRelDeg;
    public static double totalShotVelocity;
    public static double literalShotHoodRad;

    private static Timer simTimer = new Timer();
  }
  
  public static void init() {
    TalonFXConfiguration hoodPivotConfig = new TalonFXConfiguration();
    hoodPivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    hoodPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    hoodPivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
    hoodPivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    hoodPivotConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

    hoodPivotConfig.Slot0.kS = 0.55;
    hoodPivotConfig.Slot0.kV = 0.45;
    hoodPivotConfig.Slot0.kA = 0.0;
    hoodPivotConfig.Slot0.kG = 0.0;
    hoodPivotConfig.Slot0.kP = 12.0;
    hoodPivotConfig.Slot0.kI = 0.0;
    hoodPivotConfig.Slot0.kD = 0.0;

    hoodPivotConfig.Feedback.RotorToSensorRatio = 1;
    hoodPivotConfig.Feedback.SensorToMechanismRatio = 6.75;
    hoodPivotConfig.Feedback.FeedbackRotorOffset = 0.0;

    hoodPivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hoodPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hoodPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    hoodPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    hoodPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.77;
    hoodPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    hoodPivotConfig.MotionMagic.MotionMagicAcceleration = 15;
    hoodPivotConfig.MotionMagic.MotionMagicCruiseVelocity = 15;

    mHoodPivotMotor.getConfigurator().apply(hoodPivotConfig);

    mHoodPivotMotor.setPosition(0.0);

    TalonFXConfiguration turretAzimuthConfig = new TalonFXConfiguration();
    turretAzimuthConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    turretAzimuthConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretAzimuthConfig.CurrentLimits.SupplyCurrentLimit = 70;
    turretAzimuthConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    turretAzimuthConfig.CurrentLimits.SupplyCurrentLowerTime = 0.75;

    turretAzimuthConfig.Slot0.kP = 0.7;
    turretAzimuthConfig.Slot0.kI = 0;
    turretAzimuthConfig.Slot0.kD = 0.0015;
    turretAzimuthConfig.Slot0.kS = 0.3;
    turretAzimuthConfig.Slot0.kV = 0.101;
    turretAzimuthConfig.Slot0.kA = 0.001;
    turretAzimuthConfig.Slot0.kG = 0;

    turretAzimuthConfig.MotionMagic.MotionMagicAcceleration = 175;
    turretAzimuthConfig.MotionMagic.MotionMagicCruiseVelocity = 300;
    turretAzimuthConfig.MotionMagic.MotionMagicJerk = 0.0;

    turretAzimuthConfig.Feedback.SensorToMechanismRatio = 1 / 1;
    turretAzimuthConfig.Feedback.RotorToSensorRatio = 1 / 1;

    turretAzimuthConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretAzimuthConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    turretAzimuthConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    turretAzimuthConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    turretAzimuthConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 20.5;
    turretAzimuthConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -20.5;

    turretAzimuthConfig.ClosedLoopGeneral.ContinuousWrap = false;

    mAzimuthTurretMotor.getConfigurator().apply(turretAzimuthConfig);
    mAzimuthTurretMotor.setPosition(0.0);
  }

  /**
   * Commands the hood pivot motor to the calculated optimal angle.
   * * <p>Uses Motion Magic to provide a smooth trapezoidal velocity profile 
   * to the hood's position setpoint.</p>
   */
  private static void setHoodAngle() {    
    double angle = SimulationObjects.desiredHoodAngleRobotRelDeg;

    if (angle > Constants.Hood.MAX_HOOD_ANGLE_DEG) {
      angle = Constants.Hood.MAX_HOOD_ANGLE_DEG;
    } else if (angle < Constants.Hood.BASE_HOOD_ANGLE_DEG) {
      angle = Constants.Hood.BASE_HOOD_ANGLE_DEG;
    }

    mHoodPivotMotor.setControl(new MotionMagicVoltage((angle - Constants.Hood.BASE_HOOD_ANGLE_DEG) / Constants.Hood.DEGREE_RATIO).withEnableFOC(true));

    Logger.recordOutput("Turret/ setAngleForHood", SimulationObjects.desiredHoodAngleRobotRelDeg - Constants.Hood.BASE_HOOD_ANGLE_DEG);
  }

  /**
   * Commands the turret azimuth motor to the calculated robot-relative angle.
   * * <p>This method converts the calculated radians into motor rotations 
   * before sending the signal to the TalonFX via Motion Magic.</p>
   */
  private static void setAzimuthAngle() {
    double setAngle = (SimulationObjects.desiredTurretAngleRobotRelRad / TurretConstants.azimuthRotationstoRadians);    
    mAzimuthTurretMotor.setControl(new MotionMagicVoltage(setAngle).withEnableFOC(true));

    Logger.recordOutput("Turret/ setAngleAzimuth", setAngle);
  }

  /**
   * Tells the motor to return to the zero position.
   */
  public static void setAzimuthZero() {
    mAzimuthTurretMotor.setControl(new MotionMagicVoltage(0).withEnableFOC(true));
  }

  /**
   * Retrieves the current rotational position of the turret azimuth.
   * * @return The current position of the azimuth motor in rotations.
   */
  private static double getAzimuthAngle() {
    return mAzimuthTurretMotor.getPosition().getValueAsDouble();
  }

  /**
   * Retrieves the current rotational error of the turret azimuth.
   * * @return The current positional error of the azimuth motor in rotations.
   */
  public static double getAzimuthError() {
    return (SimulationObjects.desiredTurretAngleRobotRelRad / TurretConstants.azimuthRotationstoRadians) - getAzimuthAngle();
  }

  /**
   * Retrieves the current rotational position of the turret azimuth.
   * * @return The current position of the azimuth motor in degrees.
   */
  private static double getAzimuthAngleRad() {
    return mAzimuthTurretMotor.getPosition().getValueAsDouble() / TurretConstants.azimuthRotationstoRadians;
  }

  public static double getAzimuthAngularVelocity() {
    return mAzimuthTurretMotor.getVelocity().getValueAsDouble();
  }

  /** 
   * 
   * The fully abstracted method that tracks the turret azimuth and hood, no nonsense and all setup already.
   * Call to update the turret position.
   */
  public static void turretTrackHub() {
    simulateTurretAngle(Swerve.getPose(), 
                        Constants.Hood.TURRET_ROBOT_OFFSET,
                        Constants.Hood.HUB_POSE,
                        Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, 
                        Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(),
                        Swerve.getFieldSpeeds(),
                        Swerve.getLoopLatencySec());
    
    setHoodAngle();
    setAzimuthAngle();
  }

  /** 
   * The fully abstracted method that tracks the turret to the Hub without the Hood angle changing., no nonsense and all setup already.
   * Call to update the turret position.
   */
  public static void turretTrackHubAzimuth() {
    simulateTurretAngle(Swerve.getPose(), 
                        Constants.Hood.TURRET_ROBOT_OFFSET,
                        Constants.Hood.HUB_POSE,
                        Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, 
                        Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(),
                        Swerve.getFieldSpeeds(),
                        Swerve.getLoopLatencySec());
    
    setAzimuthAngle();
  }

  /** 
   * The fully abstracted method that tracks the turret to the respective alliance pass poses, no nonsense and all setup already.
   * Call to update the turret position.
   */
  public static void turretTrackPassPose() {
    Translation2d passPose = (Swerve.getPose().getY() <= 4) ? Constants.Hood.PASS_LOWER.toTranslation2d() : Constants.Hood.PASS_UPPER.toTranslation2d();

    simulateTurretAngle(Swerve.getPose(), 
                        Constants.Hood.TURRET_ROBOT_OFFSET,
                        passPose, 
                        0.5,
                        Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(),
                        Swerve.getFieldSpeeds(),
                        Swerve.getLoopLatencySec());
    
    setHoodAngle();
    setAzimuthAngle();
  }

  /** 
   * The fully abstracted method that tracks the turret azimuth to respective alliance pass poses, no nonsense and all setup already.
   * Call to update the turret position.
   */
  public static void turretTrackPassPoseAzimuth() {
    Translation2d passPose = (Swerve.getPose().getY() <= 4) ? Constants.Hood.PASS_LOWER.toTranslation2d() : Constants.Hood.PASS_UPPER.toTranslation2d();

    simulateTurretAngle(Swerve.getPose(), 
                        Constants.Hood.TURRET_ROBOT_OFFSET,
                        passPose, 
                        0.5,
                        Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(),
                        Swerve.getFieldSpeeds(),
                        Swerve.getLoopLatencySec());
    
    setHoodAngle();
    setAzimuthAngle();
  }

  /** 
   * The fully abstracted method that tracks the turret to a specified Translation2d.
   * Call to update the turret position.
   */
  public static void turretTrackPassPose(Translation2d poseToTrack) {
    simulateTurretAngle(Swerve.getPose(), 
                        Constants.Hood.TURRET_ROBOT_OFFSET,
                        poseToTrack, 
                        0,
                        Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(),
                        Swerve.getFieldSpeeds(),
                        Swerve.getLoopLatencySec());
    
    setHoodAngle();

    setAzimuthAngle();
    
    Logger.recordOutput("Turret/ Realoutputs/ Turret Azimuth", getAzimuthAngle());
  }

  /** 
   * Calculates the optimal target angle closest to the current position 
   * that respects the physical soft limits of the turret. Right now it
   * is set to respect [-180, 180].
   * 
   * @param inputAngle The desired robot-relative angle in degrees.
   * @return The optimized motor setpoint in degrees.
   */
  private static double wrapAngle(double inputAngle) {

    // safetyclamp
    if (inputAngle > TurretConstants.maxPositiveTurnAngle) {
      inputAngle -= 2 * Math.PI;
    } else if (inputAngle < TurretConstants.maxNegaitveTurnAngle) {
      inputAngle += 2 * Math.PI;
    }

    Logger.recordOutput("input Angle", inputAngle);

    return inputAngle;
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
  private static double sampleVelocity(double d, double h) {
    double velocity = Math.sqrt(
      Constants.Hood.G * (Math.sqrt(d * d + h * h) + h)) // the minimum line. go below this velocity and we will hit the SIDE.
       + 0.5 + 4 * Math.pow(Math.E, -(1 * d)); // the offset line, adjust as desired
        
    return velocity;
  }

  /**
   * Calculates the required launch angle for a 2D projectile to hit a target.
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
  private static double calculateHoodAngle(double v, double d, double h) {
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
  private static double calculateLaunchAngleRad(double v, double d, double h) {
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
   * @param currTranslation Current robot pose on the field
   * @param targetPose Original target position (hub center)
   * @param transformationInMeters Distance to shift the target along the bearing
   * @return Transformed target translation used for aiming
   */
  private static Translation2d transformTarget(Translation2d currTranslation, Translation2d targetPose, double transformationInMeters) {
    Translation2d deltaTranslation = targetPose.minus(currTranslation);
    double radAngle = Math.atan2(deltaTranslation.getY(), deltaTranslation.getX());
    return new Translation2d(targetPose.getX() + transformationInMeters * Math.cos(radAngle), targetPose.getY() + transformationInMeters * Math.sin(radAngle));
  }

  /**
   * Calculates the field-relative pose of the turret by applying a robot-relative 
   * offset to the current robot field pose.
   * * <p>This method accounts for the robot's current heading (rotation) to ensure 
   * the physical offset of the turret (e.g., if the turret is mounted 10cm behind 
   * the robot's center) is correctly translated into field coordinates.</p>
   *
   * @param robotFieldPose      The current 2D pose of the robot chassis relative to the field.
   * @param turretRelativeOffset The physical translation of the turret pivot point 
   * relative to the robot's center (Robot Frame).
   * @return A {@link Pose2d} representing the turret's position on the field and its 
   * current orientation (matching the robot's rotation).
   */
  private static Pose2d getTurretPosition(Pose2d robotFieldPose, Translation2d turretRelativeOffset) {
      Translation2d turretFieldOffset = turretRelativeOffset.rotateBy(robotFieldPose.getRotation());
      Translation2d turretFieldPosition = robotFieldPose.getTranslation().plus(turretFieldOffset);

      return new Pose2d(turretFieldPosition, robotFieldPose.getRotation());
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
   * @param turretRelative     The relative position of the turret to the robot. The frame of refernce is in meters. 
   * X axis is perpendicular to the front face, Y is parallel to the front face.
   * @param targetPose         The static 2D field position of the goal/hub (meters).
   * @param targetHeightRelativeBot The exit-point relative height from the target to the hood (meters).
   * @param robotFieldYaw      The current rotation of the robot relative to the field (radians).
   * @param robotYawRate       The angular velocity of the robot (rad/s).
   * @param fieldRobotSpeeds   The current velocity vector of the chassis in field-relative coordinates (m/s).
   * @param loopLatencySec     The time delay of the control loop (seconds) for preemptive compensation.
   * @param numLoops           The number of loops to iterate on and estimate the turret constants.
   * * @implNote 
   * <b>Frame of Reference:</b>
   * <ul>
   * <li>The "Fake Target" is calculated in the <b>Field Coordinate Frame</b>.</li>
   * <li>The final Turret Angle is converted to the <b>Robot Coordinate Frame</b> (Robot-Relative).</li>
   * <li>The Hood Angle is relative to the <b>Horizontal Plane</b> (0° = Floor Parallel).</li>
   * </ul>
   */
  public static void simulateTurretAngle(Pose2d currPose, 
                                          Translation2d turretRelative, 
                                          Translation2d targetPose, 
                                          double targetHeightRelativeBot,
                                          double robotFieldYaw,
                                          double robotYawRate,
                                          ChassisSpeeds fieldRobotSpeeds,
                                          double loopLatencySec) 
  {
    Pose2d turretPose = getTurretPosition(currPose, turretRelative);
    Translation2d transformedTarget = transformTarget(turretPose.getTranslation(), targetPose, 0.25);
    
    double xSpeeds = fieldRobotSpeeds.vxMetersPerSecond;
    double ySpeeds = fieldRobotSpeeds.vyMetersPerSecond;

    double dx = transformedTarget.getX() - turretPose.getX();
    double dy = transformedTarget.getY() - turretPose.getY();
    double currentR = Math.sqrt(dx * dx + dy * dy);

    double currentV = sampleVelocity(currentR, targetHeightRelativeBot);
    double currentTheta = calculateLaunchAngleRad(currentV, currentR, targetHeightRelativeBot);
    double timeOFlight = currentR / (currentV * Math.cos(currentTheta));

    Translation2d targetOffset = new Translation2d(xSpeeds, ySpeeds).times(timeOFlight);
    Translation2d imaginaryTarget = transformedTarget.minus(targetOffset);
        
    double newDX = imaginaryTarget.getX() - turretPose.getX();
    double newDY = imaginaryTarget.getY() - turretPose.getY();
    double newR = Math.sqrt(newDX * newDX + newDY * newDY);
    double newV = sampleVelocity(newR, targetHeightRelativeBot);
    
    double newTheta = calculateLaunchAngleRad(newV, newR, targetHeightRelativeBot);
    timeOFlight = newR / newV * Math.cos(newTheta);
    
    SimulationObjects.desiredTurretAngleRobotRelRad 
      = wrapAngle(Math.atan2(newDY, newDX) 
        - robotFieldYaw 
        - (robotYawRate * Constants.Hood.YAW_COMPENSATION_LATENCY_MS / 1000));
    SimulationObjects.totalShotVelocity = newV;
    SimulationObjects.desiredHoodAngleRobotRelDeg = calculateHoodAngle(newV, newR, targetHeightRelativeBot);
    SimulationObjects.literalShotHoodRad = newTheta;

    Logger.recordOutput("Turret/ Turret Sim/ Turret 3D Pose", new Pose3d(turretPose.getX(), turretPose.getY(), 0, new Rotation3d(0 , 0, SimulationObjects.desiredTurretAngleRobotRelRad + robotFieldYaw)));
}

  /**
   * These two methods are only used for simulation. Can be deleted afterwards.
   */
  private static void launchFuel() {

    if (Robot.isReal()) {
      return;
    }

    SimulationObjects.simTimer.start();

    if (!SimulationObjects.simTimer.hasElapsed(0.2)) {
      return;
    }

    Translation2d turret = getTurretPosition(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET).getTranslation();

    Translation3d initialPosition = new Translation3d(turret).plus(new Translation3d(0, 0, 0.3));
    FuelSim.getInstance().spawnFuel(initialPosition, launchVectorSim().plus(
      new Translation3d(Swerve.getFieldSpeeds().vxMetersPerSecond, Swerve.getFieldSpeeds().vyMetersPerSecond, 0)));

    SimulationObjects.simTimer.reset();
  }

  /**
   * These two methods are only used for simulation. Can be deleted afterwards.
   */
  private static Translation3d launchVectorSim() {
    double hoodAngleRad = SimulationObjects.literalShotHoodRad;
    double turretThetaRad = SimulationObjects.desiredTurretAngleRobotRelRad + Swerve.getYawAsRadians(); // make this field relative again

    double z = SimulationObjects.totalShotVelocity * Math.sin(hoodAngleRad);
    double x = SimulationObjects.totalShotVelocity * Math.cos (hoodAngleRad) * Math.cos(turretThetaRad);
    double y = SimulationObjects.totalShotVelocity * Math.cos (hoodAngleRad) * Math.sin(turretThetaRad);

    Translation3d shotVec = new Translation3d(x, y, z);

    Logger.recordOutput("Turret/ Turret Sim/ Shot Vector", shotVec);
    return shotVec;
  }
}