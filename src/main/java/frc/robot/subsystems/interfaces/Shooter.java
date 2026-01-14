package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants;
import frc.robot.lib.util.Conversions;

public class Shooter {

  private static final TalonFX mBackShooterMotor =
      new TalonFX(Constants.Shooter.BACK_FLYWHEEL_MOTOR);
  private static final TalonFX mFrontMasterShooterMotor =
      new TalonFX(Constants.Shooter.FRONT_MASTER_FLYWHEEL_MOTOR);  
  private static final TalonFX mFrontFollowerShooterMotor =
      new TalonFX(Constants.Shooter.FRONT_FOLLOWER_FLYWHEEL_MOTOR);

  public static final StatusSignal<AngularVelocity> mBackMotorVelo = mBackShooterMotor.getVelocity();
  public static final StatusSignal<AngularVelocity> mFrontMotorVelo = mFrontMasterShooterMotor.getVelocity();

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.0);
  public static final double AMP_DUTY_CYCLE = -0.5;
  public static final double OUUTake = 0.25;
  public static final double SPEED_PASS_THRESH = 0.95;
  public static final double SHOOTER_RATIO = 0.5;

  public static void init() {

    TalonFXConfiguration frontShooterMotorConfig = new TalonFXConfiguration();

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    frontShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    frontShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;

    frontShooterMotorConfig.Slot0.kS = 9.75;
    frontShooterMotorConfig.Slot0.kV = 0.132;
    frontShooterMotorConfig.Slot0.kA = 0.31;
    frontShooterMotorConfig.Slot0.kP = 6.0;
    frontShooterMotorConfig.Slot0.kI = 0.0;
    frontShooterMotorConfig.Slot0.kD = 0.0;

    frontShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 160.0;

    frontShooterMotorConfig.Feedback.SensorToMechanismRatio = 24.0 / 36.0;

    frontShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mFrontMasterShooterMotor.getConfigurator().apply(frontShooterMotorConfig);
    mFrontFollowerShooterMotor.getConfigurator().apply(frontShooterMotorConfig);

    TalonFXConfiguration backShooterMotorConfig = new TalonFXConfiguration();

    backShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    backShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    backShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    backShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    backShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    backShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    backShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;

    backShooterMotorConfig.Slot0.kS = 7.8;
    backShooterMotorConfig.Slot0.kV = 0.134;
    backShooterMotorConfig.Slot0.kA = 0.288;
    backShooterMotorConfig.Slot0.kP = 6.0;
    backShooterMotorConfig.Slot0.kI = 0.0;
    backShooterMotorConfig.Slot0.kD = 0.0;

    backShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    backShooterMotorConfig.Feedback.SensorToMechanismRatio = 24.0 / 36.0;

    backShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    backShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mBackShooterMotor.getConfigurator().apply(backShooterMotorConfig);
  }

  public static double getfrontSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mFrontMasterShooterMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  public static double getbackSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mBackShooterMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  // speed in meters per second
  public static void setShootSpeed(double speed) {
    // https://en.wikipedia.org/wiki/Angular_velocity
    double back = Conversions.linearSpeedToRotationalSpeed(speed * 0.5, (WHEEL_DIAMETER / 2.0));
    double front = Conversions.linearSpeedToRotationalSpeed(speed, (WHEEL_DIAMETER / 2.0));
    mBackShooterMotor.setControl(
        new MotionMagicVelocityVoltage(back)
            .withUpdateFreqHz(1000.0)
            .withEnableFOC(true));
    mFrontMasterShooterMotor.setControl(
        new MotionMagicVelocityVoltage(front)
        .withUpdateFreqHz(1000.0)
        .withEnableFOC(true));
    mFrontFollowerShooterMotor.setControl(
      new Follower(
        Constants.Shooter.FRONT_FOLLOWER_FLYWHEEL_MOTOR, 
        MotorAlignmentValue.Aligned));  
  }

  public static void stopAll() {
    mBackShooterMotor.setControl(new DutyCycleOut(0));
    mFrontMasterShooterMotor.setControl(new DutyCycleOut(0));
    mFrontFollowerShooterMotor.setControl(new DutyCycleOut(0)); 
  }

  public static boolean readyToShoot() {
    //TODO
    return false;
  }


  /** 
   * 
   * @param v The velocity as the piece exits the robot's shooter.
   * @param d The distance from the exit point to the target.
   * @param h The height from the exit point to the target.
   * @return angle in degrees that the robot should aim at.
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
}
