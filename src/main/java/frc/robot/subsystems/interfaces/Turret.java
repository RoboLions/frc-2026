package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Robot;
import frc.robot.lib.util.FuelSim;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class Turret {
  private static final TalonFX mHoodPivotMotor =
    new TalonFX(Constants.CAN_IDS.HOOD_PIVOT_MOTOR, "CANexternal");
  private static final TalonFX mAzimuthTurretMotor = 
    new TalonFX(Constants.CAN_IDS.TURRET_AZIMUTH_MOTOR, "CANexternal");

  private class TurretConstants {
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
    hoodPivotConfig.CurrentLimits.StatorCurrentLimit = 60;
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
    // mHoodPivotMotor.setPosition(0.0);

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

    turretAzimuthConfig.MotionMagic.MotionMagicAcceleration = 300;
    turretAzimuthConfig.MotionMagic.MotionMagicCruiseVelocity = 225;
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
    // mAzimuthTurretMotor.setPosition(0.0);
  }

  private static void setHoodAngle() {    
    double angle = MathUtil.clamp(
        SimulationObjects.desiredHoodAngleRobotRelDeg, 
        Constants.Hood.BASE_HOOD_ANGLE_DEG, 
        Constants.Hood.MAX_HOOD_ANGLE_DEG
    );

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

  public static double getAzimuthAngularVelocity() {
    return mAzimuthTurretMotor.getVelocity().getValueAsDouble();
  }

  public static void turretTrackHub() {
    simulateTurretAngle(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET, Constants.Hood.HUB_POSE,
                        Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(), Swerve.getFieldSpeeds(), false);
    setHoodAngle();
    setAzimuthAngle();
  }

  public static void turretTrackHubAzimuth() {
    simulateTurretAngle(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET, Constants.Hood.HUB_POSE,
                        Constants.Hood.HEIGHT_FROM_BOT_TO_TARGET, Swerve.getYawAsRadians(), 
                        Swerve.getYawRateAsRad(), Swerve.getFieldSpeeds(), false);
    setAzimuthAngle();
  }

  public static void turretTrackPassPose() {
    Translation2d passPose = (Swerve.getPose().getY() <= 4) ? Constants.Hood.PASS_LOWER.toTranslation2d() : Constants.Hood.PASS_UPPER.toTranslation2d();
    // Use low arc for passing (true)
    simulateTurretAngle(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET, passPose, 
                        -0.5, Swerve.getYawAsRadians(), Swerve.getYawRateAsRad(), Swerve.getFieldSpeeds(), true);
    setHoodAngle();
    setAzimuthAngle();
  }

  public static void turretTrackPassPoseAzimuth() {
    Translation2d passPose = (Swerve.getPose().getY() <= 4) ? Constants.Hood.PASS_LOWER.toTranslation2d() : Constants.Hood.PASS_UPPER.toTranslation2d();
    // Use low arc for passing (true)
    simulateTurretAngle(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET, passPose, 
                        0.0, Swerve.getYawAsRadians(), Swerve.getYawRateAsRad(), Swerve.getFieldSpeeds(), true);
    setHoodAngle();
    setAzimuthAngle();
  }

  public static void turretTrackPassPose(Translation2d poseToTrack) {
    // Use low arc for passing (true)
    simulateTurretAngle(Swerve.getPose(), Constants.Hood.TURRET_ROBOT_OFFSET, poseToTrack, 
                        0.0, Swerve.getYawAsRadians(), Swerve.getYawRateAsRad(), Swerve.getFieldSpeeds(), true);
    setHoodAngle();
    setAzimuthAngle();
    Logger.recordOutput("Turret/ Realoutputs/ Turret Azimuth", getAzimuthAngle());
  }
  
  private static double sampleVelocity(double d, double h) {
    return Math.sqrt(Constants.Hood.G * (Math.hypot(d, h) + h)) + 1.0 + 4.0 * Math.exp(-d);
  }

  /**
   * Consolidates both original Launch and Hood calculation math.
   * Calculates the trajectory angle for a 2D projectile to hit a target.
   * * @param useLowArc If true, uses the lower discriminant root (flatter shot, passing).
   * If false, uses the upper discriminant root (lobbed shot, shooting).
   * @return angle in Rad relative to the horizontal plane. Double.NaN if no physical solution.
   */
  private static double calculateTrajectoryAngleRad(double v, double d, double h, boolean useLowArc) {
    final double g = Constants.Hood.G;
    double v2 = v * v;
    double v4 = v2 * v2;
    double d2 = d * d;
    
    double discriminant = v4 - g * (g * d2 + 2.0 * h * v2);
    if (discriminant < 0.0 || d == 0.0) {
      return Double.NaN; 
    }

    double root = Math.sqrt(discriminant);
    //lower arc requires subtracting the root, high arc (lob) adds it.
    double numerator = useLowArc ? (v2 - root) : (v2 + root);
    return Math.atan(numerator / (g * d));
  } 

  private static Translation2d transformTarget(Translation2d currTranslation, Translation2d targetPose, double transformationInMeters) {
    Translation2d delta = targetPose.minus(currTranslation);
    double distance = delta.getNorm();
    
    if (distance == 0.0) return targetPose;
    return targetPose.plus(delta.times(transformationInMeters / distance));
  }

  private static Pose2d getTurretPosition(Pose2d robotFieldPose, Translation2d turretRelativeOffset) {
      Translation2d turretFieldOffset = turretRelativeOffset.rotateBy(robotFieldPose.getRotation());
      return new Pose2d(robotFieldPose.getTranslation().plus(turretFieldOffset), robotFieldPose.getRotation());
  }

  /**
   * Simulates and calculates turret and hood kinematics.
   * * @param isPassing If true, commands the trajectory algorithm to utilize the lower arc solution.
   */
  public static void simulateTurretAngle(Pose2d currPose, Translation2d turretRelative, 
                                         Translation2d targetPose, double targetHeightRelativeBot,
                                         double robotFieldYaw, double robotYawRate,
                                         ChassisSpeeds fieldRobotSpeeds, boolean isPassing) 
  {
    Pose2d turretPose = getTurretPosition(currPose, turretRelative);
    Translation2d transformedTarget = transformTarget(turretPose.getTranslation(), targetPose, 0.25);
    
    double xSpeeds = fieldRobotSpeeds.vxMetersPerSecond;
    double ySpeeds = fieldRobotSpeeds.vyMetersPerSecond;

    double dx = transformedTarget.getX() - turretPose.getX();
    double dy = transformedTarget.getY() - turretPose.getY();
    double currentR = Math.hypot(dx, dy);

    double currentV = sampleVelocity(currentR, targetHeightRelativeBot);
    double currentTheta = calculateTrajectoryAngleRad(currentV, currentR, targetHeightRelativeBot, isPassing);
    
    double timeOfFlight = currentR / (currentV * Math.cos(currentTheta));

    Translation2d targetOffset = new Translation2d(xSpeeds, ySpeeds).times(timeOfFlight);
    Translation2d imaginaryTarget = transformedTarget.minus(targetOffset);
        
    double newDX = imaginaryTarget.getX() - turretPose.getX();
    double newDY = imaginaryTarget.getY() - turretPose.getY();
    double newR = Math.hypot(newDX, newDY);
    double newV = sampleVelocity(newR, targetHeightRelativeBot);
    
    double newTheta = calculateTrajectoryAngleRad(newV, newR, targetHeightRelativeBot, isPassing);
    timeOfFlight = newR / (newV * Math.cos(newTheta));
    
    //wpilib method to confine bounds cleanly between [-PI, PI]
    SimulationObjects.desiredTurretAngleRobotRelRad = MathUtil.angleModulus(
        Math.atan2(newDY, newDX) 
        - robotFieldYaw  
        - (robotYawRate * Constants.Hood.YAW_COMPENSATION_LATENCY_MS / 1000.0)
    );
    
    SimulationObjects.totalShotVelocity = newV;
    SimulationObjects.literalShotHoodRad = newTheta;
    
    if (!Double.isNaN(newTheta)) {
        SimulationObjects.desiredHoodAngleRobotRelDeg = Constants.Hood.THETA_ANGLE_FROM_SHOOTER - Math.toDegrees(newTheta);
    }

    if (Robot.isSimulation()) {
      launchFuel();
    }
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