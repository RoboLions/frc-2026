package frc.robot.subsystems.interfaces;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class Shooter {

  private static final TalonFX mMasterFlywheelMotor =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_MASTER, "CANexternal");
  private static final TalonFX mFollowerFlywheelMotor1 =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_FOLLOWER_UPPER_RIGHT, "CANexternal");    
  private static final TalonFX mFollowerFlywheelMotor2 =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_FOLLOWER_LOWER_LEFT, "CANexternal");  
  private static final TalonFX mFollowerFlywheelMotor3 =
      new TalonFX(Constants.CAN_IDS.FLYWHEEL_MOTOR_FOLLOWER_LOWER_RIGHT, "CANexternal");  

  private static ArrayList<ShotPoint> VELOCITY_LOOKUP_TABLE = new ArrayList<>();
  private static ArrayList<ShotPoint> PASSING_LOOKUP_TABLE = new ArrayList<>();

  private static class ShotPoint {
    double distance; //meters
    double velocity; //mps

    public ShotPoint(double dist, double vel) {
      this.distance = dist;
      this.velocity = vel;
    }
  }

  public static void init() {
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(1.7, 38.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(2.09, 42.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(2.51, 45.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(3.0, 47.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(3.5, 50.0));
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(3.81, 52)); //B
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(4.2, 54.0)); //B
    VELOCITY_LOOKUP_TABLE.add(new ShotPoint(4.5, 56.0)); //B

    PASSING_LOOKUP_TABLE.add(new ShotPoint(4.0, 40));
    PASSING_LOOKUP_TABLE.add(new ShotPoint(8.5, 70));

    TalonFXConfiguration shooterMotorConfig = new TalonFXConfiguration();

    shooterMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 800;
    shooterMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -800;

    shooterMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    shooterMotorConfig.CurrentLimits.StatorCurrentLimit = 160;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
    shooterMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1.0;

    shooterMotorConfig.Slot0.kS = 6;
    shooterMotorConfig.Slot0.kV = 0;
    shooterMotorConfig.Slot0.kA = 0;
    shooterMotorConfig.Slot0.kP = 10;
    shooterMotorConfig.Slot0.kI = 0.0;
    shooterMotorConfig.Slot0.kD = 0.0;

    shooterMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    shooterMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    shooterMotorConfig.Feedback.SensorToMechanismRatio = 1 / 1;
    shooterMotorConfig.Feedback.RotorToSensorRatio = 1 / 1;

    shooterMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    mMasterFlywheelMotor.getConfigurator().apply(shooterMotorConfig);
    mFollowerFlywheelMotor1.getConfigurator().apply(shooterMotorConfig);
    mFollowerFlywheelMotor2.getConfigurator().apply(shooterMotorConfig);
    mFollowerFlywheelMotor3.getConfigurator().apply(shooterMotorConfig);

    mFollowerFlywheelMotor1.setControl(
        new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(100));
    
    mFollowerFlywheelMotor2.setControl(
        new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Aligned)
            .withUpdateFreqHz(100));
    
    mFollowerFlywheelMotor3.setControl(
        new Follower(mMasterFlywheelMotor.getDeviceID(), MotorAlignmentValue.Opposed)
            .withUpdateFreqHz(100));
  }

  public static void interpolateAndShoot(double currentDistance) {
    double velocity = getInterpolatedVelocity(currentDistance);
    setShootSpeed(velocity);

    Logger.recordOutput("Shooter/ ShootSpeed (RPS)", velocity);
    Logger.recordOutput("Shooter/ Dist-To-Target (M)", currentDistance);
  }

  public static void interpoleateAndPass(double currentDistance) {
    double velocity = getPassingVelocity(currentDistance);
    setShootSpeed(velocity);

    Logger.recordOutput("Shooter/ Passing-Speed (RPS)", velocity);
    Logger.recordOutput("Shooter/ Dist-To-PassTarget (M)", currentDistance);
  }
  
  private static double getInterpolatedVelocity(double currentDistance) {
      if (VELOCITY_LOOKUP_TABLE.isEmpty()) return 0.0;

      if (currentDistance <= VELOCITY_LOOKUP_TABLE.get(0).distance) {
          return VELOCITY_LOOKUP_TABLE.get(0).velocity;
      }

      for (int i = 0; i < VELOCITY_LOOKUP_TABLE.size() - 1; i++) {
          ShotPoint p1 = VELOCITY_LOOKUP_TABLE.get(i);
          ShotPoint p2 = VELOCITY_LOOKUP_TABLE.get(i + 1);

          if (currentDistance <= p2.distance) {
              //linear Interpolation formula: y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)
              double t = (currentDistance - p1.distance) / (p2.distance - p1.distance);
              return p1.velocity + t * (p2.velocity - p1.velocity);
          }
      }

      return VELOCITY_LOOKUP_TABLE.get(VELOCITY_LOOKUP_TABLE.size() - 1).velocity;
  }

  private static double getPassingVelocity(double currentDistance) {
      if (PASSING_LOOKUP_TABLE.isEmpty()) return 0.0;

      if (currentDistance <= PASSING_LOOKUP_TABLE.get(0).distance) {
          return PASSING_LOOKUP_TABLE.get(0).velocity;
      }

      for (int i = 0; i < PASSING_LOOKUP_TABLE.size() - 1; i++) {
          ShotPoint p1 = PASSING_LOOKUP_TABLE.get(i);
          ShotPoint p2 = PASSING_LOOKUP_TABLE.get(i + 1);

          if (currentDistance <= p2.distance) {
              //linear Interpolation formula: y = y1 + ((x - x1) / (x2 - x1)) * (y2 - y1)
              double t = (currentDistance - p1.distance) / (p2.distance - p1.distance);
              return p1.velocity + t * (p2.velocity - p1.velocity);
          }
      }

      return PASSING_LOOKUP_TABLE.get(PASSING_LOOKUP_TABLE.size() - 1).velocity;
  }

  // speed in meters per second
  private static void setShootSpeed(double setSpeed) {
    mMasterFlywheelMotor.setControl(
        new VelocityTorqueCurrentFOC(setSpeed)
            .withUpdateFreqHz(100));
  }

  public static void idlerShooter() {
    mMasterFlywheelMotor.setControl(
        new VoltageOut(0.0).withUpdateFreqHz(20));
  }
}
