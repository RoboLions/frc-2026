package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_MASTER);
  private static final TalonFX mFollowerFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_FOLLOWER);  

  private static final double WHEEL_DIAMETER = Units.inchesToMeters(4);

  public static void init() {

    TalonFXConfiguration frontShooterMotorConfig = new TalonFXConfiguration();

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    frontShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    frontShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

    frontShooterMotorConfig.Slot0.kS = 7.95;
    frontShooterMotorConfig.Slot0.kV = 0.0825;
    frontShooterMotorConfig.Slot0.kA = 0.029;
    frontShooterMotorConfig.Slot0.kP = 11.5;
    frontShooterMotorConfig.Slot0.kI = 0.0;
    frontShooterMotorConfig.Slot0.kD = 0.0;

    frontShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 200.0;

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
    double setSpeed = Conversions.linearSpeedToRotationalSpeed(speed, (WHEEL_DIAMETER / 2.0)) * Constants.Shooter.POWER_GAIN_MULTIPLIER;

    Logger.recordOutput("Shooter/ Flywheel setspeed", setSpeed);

    mMasterFlywheelMotor.setControl(
        new VelocityTorqueCurrentFOC(setSpeed)
            .withUpdateFreqHz(500));
    
    mFollowerFlywheelMotor.setControl(
        new VelocityTorqueCurrentFOC(setSpeed)
            .withUpdateFreqHz(500));
  }

  public static void stopAll() {
    mMasterFlywheelMotor.setControl(new VoltageOut(0));
    mFollowerFlywheelMotor.setControl(new VoltageOut(0)); 
  }

  public static boolean readyToShoot() {
    //TODO
    return false;
  }



}
