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

  private static final TalonFX mFollowerFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_MASTER);
  private static final TalonFX mMasterFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_FOLLOWER);  

  public static final StatusSignal<AngularVelocity> mBackMotorVelo = mFollowerFlywheelMotor.getVelocity();
  public static final StatusSignal<AngularVelocity> mFrontMotorVelo = mMasterFlywheelMotor.getVelocity();

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

    mMasterFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
    mFollowerFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
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
    double back = Conversions.linearSpeedToRotationalSpeed(speed * 0.5, (WHEEL_DIAMETER / 2.0));
    double front = Conversions.linearSpeedToRotationalSpeed(speed, (WHEEL_DIAMETER / 2.0));
    mMasterFlywheelMotor.setControl(
        new MotionMagicVelocityVoltage(back)
            .withUpdateFreqHz(1000.0)
            .withEnableFOC(true));
    mFollowerFlywheelMotor.setControl(
      new Follower(
        Constants.CAN_IDS.FLYWHEEL_MOTOR_MASTER, 
        MotorAlignmentValue.Aligned));  
  }

  public static void stopAll() {
    mMasterFlywheelMotor.setControl(new DutyCycleOut(0));
    mFollowerFlywheelMotor.setControl(new DutyCycleOut(0)); 
  }

  public static boolean readyToShoot() {
    //TODO
    return false;
  }



}
