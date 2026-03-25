package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.lib.util.Conversions;

public class Shooter {

  private static final TalonFX mMasterFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_RIGHT, "CANexternal");
  private static final TalonFX mFollowerFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_LEFT, "CANexternal");  

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

  public static void init() {

    TalonFXConfiguration frontShooterMotorConfig = new TalonFXConfiguration();

    frontShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    frontShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 140;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    frontShooterMotorConfig.Slot0.kS = 2.3;
    frontShooterMotorConfig.Slot0.kV = 0.0007;
    frontShooterMotorConfig.Slot0.kA = 0.0015;
    frontShooterMotorConfig.Slot0.kP = 7;
    frontShooterMotorConfig.Slot0.kI = 0.0;
    frontShooterMotorConfig.Slot0.kD = 0.0;

    frontShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    frontShooterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    frontShooterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    frontShooterMotorConfig.Feedback.SensorToMechanismRatio = 1 / 1;
    frontShooterMotorConfig.Feedback.RotorToSensorRatio = 1 / 1;

    frontShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mMasterFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
    mFollowerFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
    mFollowerFlywheelMotor.setControl(new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }
  

  public static double getfrontSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mMasterFlywheelMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  public static double getbackSpeed() {
    return Conversions.rotationalSpeedToLinearSpeed(
        mMasterFlywheelMotor.getVelocity().getValueAsDouble(), (WHEEL_DIAMETER / 2.0));
  }

  // speed in meters per second
  public static void setShootSpeed(double speed) {
    // https://en.wikipedia.org/wiki/Angular_velocity
    double adjustedSpeed = speed * Constants.Shooter.POWER_GAIN_MULTIPLIER;

    double setRotationalSpeed = Conversions.linearSpeedToRotationalSpeed(adjustedSpeed, (WHEEL_DIAMETER / 2.0));

    mMasterFlywheelMotor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(setRotationalSpeed)
            .withUpdateFreqHz(75));
    
    mFollowerFlywheelMotor.setControl(
        new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(75));
  }

  public static void idlerShooter() {
    mMasterFlywheelMotor.setControl(new VoltageOut(0.5)
        .withEnableFOC(true)
        .withUpdateFreqHz(20));
    mFollowerFlywheelMotor.setControl(new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        .withUpdateFreqHz(20));
  }

  public static void stopAll() {
    mMasterFlywheelMotor.setControl(new VoltageOut(0));
    mFollowerFlywheelMotor.setControl(new VoltageOut(0));
  }
}
