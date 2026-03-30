package frc.robot.subsystems.interfaces;

import java.util.ArrayList;

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
  private static ArrayList<ShotPoint> VELOCITY_LOOKUP_TABLE = new ArrayList<>();

  private static class ShotPoint {
    double distance; //meters
    double velocity; //mps

    public ShotPoint(double dist, double vel) {
      this.distance = dist;
      this.velocity = vel;
    }
  }

  public static void init() {
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(0.25, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(0.5, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(1.0, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(1.5, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(2.0, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(2.5, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(3.0, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(3.5, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(4.0, 0.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(4.5, 0.0));

    TalonFXConfiguration frontShooterMotorConfig = new TalonFXConfiguration();

    frontShooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    frontShooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    frontShooterMotorConfig.CurrentLimits.StatorCurrentLimit = 170;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 60;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    frontShooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    frontShooterMotorConfig.Slot0.kS = 2.3;
    frontShooterMotorConfig.Slot0.kV = 0.0007;
    frontShooterMotorConfig.Slot0.kA = 0.0015;
    frontShooterMotorConfig.Slot0.kP = 7;
    frontShooterMotorConfig.Slot0.kI = 0.0;
    frontShooterMotorConfig.Slot0.kD = 0.0;

    frontShooterMotorConfig.MotionMagic.MotionMagicAcceleration = 500.0;

    frontShooterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    frontShooterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    frontShooterMotorConfig.Feedback.SensorToMechanismRatio = 1 / 1;
    frontShooterMotorConfig.Feedback.RotorToSensorRatio = 1 / 1;

    frontShooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    frontShooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mMasterFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
    mFollowerFlywheelMotor.getConfigurator().apply(frontShooterMotorConfig);
  }
  
  public static double getInterpolatedVelocity(double currentDistance) {
      if (VELOCITY_LOOKUP_TABLE.isEmpty()) return 0.0;

      //handle out-of-bounds close
      if (currentDistance <= VELOCITY_LOOKUP_TABLE.get(0).distance) {
          return VELOCITY_LOOKUP_TABLE.get(0).velocity;
      }

      //iterate through the table to find the "gap"
      for (int i = 0; i < VELOCITY_LOOKUP_TABLE.size() - 1; i++) {
          ShotPoint p1 = VELOCITY_LOOKUP_TABLE.get(i);
          ShotPoint p2 = VELOCITY_LOOKUP_TABLE.get(i + 1);

          if (currentDistance <= p2.distance) {
              //linear Interpolation formula: y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)
              double t = (currentDistance - p1.distance) / (p2.distance - p1.distance);
              return p1.velocity + t * (p2.velocity - p1.velocity);
          }
      }

      //handle out-of-bounds far
      return VELOCITY_LOOKUP_TABLE.get(VELOCITY_LOOKUP_TABLE.size() - 1).velocity;
  }

  // speed in meters per second
  public static void setShootSpeed(double speed) {
    double adjustedSpeed = speed * Constants.Shooter.POWER_GAIN_MULTIPLIER;

    double setRotationalSpeed = Conversions.linearSpeedToRotationalSpeed(adjustedSpeed, (WHEEL_DIAMETER / 2.0));

    mMasterFlywheelMotor.setControl(
        new MotionMagicVelocityTorqueCurrentFOC(setRotationalSpeed)
            .withUpdateFreqHz(50));
    
    mFollowerFlywheelMotor.setControl(
        new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(50));
  }

  public static void idlerShooter() {
    mMasterFlywheelMotor.setControl(new VoltageOut(0.5)
        .withEnableFOC(true)
        .withUpdateFreqHz(20));
    mFollowerFlywheelMotor.setControl(new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
        .withUpdateFreqHz(20));
  }
}
