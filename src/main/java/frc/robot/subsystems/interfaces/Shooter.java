package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.lib.util.Conversions;

public class Shooter {

  private static final TalonFX mLeftShooterMotor =
      new TalonFX(0);
  private static final TalonFX mRightShooterMotor =
      new TalonFX(0);

  public static final StatusSignal<AngularVelocity> mLeftMotorVelo = mLeftShooterMotor.getVelocity();
  public static final StatusSignal<AngularVelocity> mRightMotorVelo = mRightShooterMotor.getVelocity();

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.0);
  public static final double AMP_DUTY_CYCLE = -0.5;
  public static final double OUUTake = 0.25;
  public static final double SPEED_PASS_THRESH = 0.95;
  public static final double SHOOTER_RATIO = 0.5;

  private static double target_speed_right = 0.0;
  private static double target_speed_left = 0.0;

  public static void init() {

    TalonFXConfiguration leftShooterMotorConfig = new TalonFXConfiguration();

    leftShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    leftShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    leftShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    leftShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;

    leftShooterMotorConfig.Slot0.kS = 9.75;
    leftShooterMotorConfig.Slot0.kV = 0.132;
    leftShooterMotorConfig.Slot0.kA = 0.31;
    leftShooterMotorConfig.Slot0.kP = 6.0;
    leftShooterMotorConfig.Slot0.kI = 0.0;
    leftShooterMotorConfig.Slot0.kD = 0.0;

    leftShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 160.0;

    leftShooterMotorConfig.Feedback.SensorToMechanismRatio = 24.0 / 36.0;

    leftShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leftShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mLeftShooterMotor.getConfigurator().apply(leftShooterMotorConfig);

    TalonFXConfiguration rightShooterMotorConfig = new TalonFXConfiguration();

    rightShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    rightShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
    rightShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
    rightShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;

    rightShooterMotorConfig.Slot0.kS = 7.8;
    rightShooterMotorConfig.Slot0.kV = 0.134;
    rightShooterMotorConfig.Slot0.kA = 0.288;
    rightShooterMotorConfig.Slot0.kP = 6.0;
    rightShooterMotorConfig.Slot0.kI = 0.0;
    rightShooterMotorConfig.Slot0.kD = 0.0;

    rightShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    rightShooterMotorConfig.Feedback.SensorToMechanismRatio = 24.0 / 36.0;

    rightShooterMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mRightShooterMotor.getConfigurator().apply(rightShooterMotorConfig);
  }

  public static double getLeftSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mLeftShooterMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  public static double getRightSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mRightShooterMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  // speed in meters per second
  public static void setShootSpeed(double speed) {
    // https://en.wikipedia.org/wiki/Angular_velocity
    double right = Conversions.linearSpeedToRotationalSpeed(speed * 0.5, (WHEEL_DIAMETER / 2.0));
    double left = Conversions.linearSpeedToRotationalSpeed(speed, (WHEEL_DIAMETER / 2.0));
    mRightShooterMotor.setControl(
        new MotionMagicVelocityVoltage(right)
            .withUpdateFreqHz(1000.0)
            .withEnableFOC(true));
    mLeftShooterMotor.setControl(
        new MotionMagicVelocityVoltage(left)
        .withUpdateFreqHz(1000.0)
        .withEnableFOC(true));
  }

  // public static void aPassShoot() {
  //   double exitVelo =
  //       Math.sqrt(
  //           (9.81 * Swerve.getDistToAPass())
  //               / Math.sin(2 * Rotation2d.fromDegrees(45).getRadians()));
  //   double targetSpeed = exitVelo / Arm.PASS_POWER_LOSS_MULTIPLIER;
  //   setShootSpeed(targetSpeed);
  // }

  // public static void cPassShoot() {
  //   double exitVelo =
  //       Math.sqrt(
  //           (9.81 * Swerve.getDistToCPass())
  //               / Math.sin(2 * Rotation2d.fromDegrees(45).getRadians()));
  //   double targetSpeed = exitVelo / Arm.CPASS_POWER_LOSS_MULTIPLIER;
  //   setShootSpeed(targetSpeed);
  // }

  public static void setAmpSpeed() {
    mLeftShooterMotor.setControl(new DutyCycleOut(AMP_DUTY_CYCLE).withEnableFOC(true));
    mRightShooterMotor.setControl(new DutyCycleOut(AMP_DUTY_CYCLE).withEnableFOC(true));
  }

  public static void setOuttakeSpeed() {
    mLeftShooterMotor.setControl(new DutyCycleOut(OUUTake).withEnableFOC(true));
    mRightShooterMotor.setControl(new DutyCycleOut(OUUTake).withEnableFOC(true));
  }

  public static void stopAll() {
    mLeftShooterMotor.setControl(new DutyCycleOut(0));
    mRightShooterMotor.setControl(new DutyCycleOut(0));
  }

  public static void set(double speed) {
    mLeftShooterMotor.setControl(new DutyCycleOut(speed));
    mRightShooterMotor.setControl(new DutyCycleOut(speed));
  }

  public static boolean readyToShoot() {
    double speed_thresh_right = target_speed_right * SPEED_PASS_THRESH;
    double speed_thresh_left = target_speed_left * SPEED_PASS_THRESH;
    return ((mRightMotorVelo.getValueAsDouble() > speed_thresh_right)
        && (mLeftMotorVelo.getValueAsDouble() > speed_thresh_left));
  }
}
