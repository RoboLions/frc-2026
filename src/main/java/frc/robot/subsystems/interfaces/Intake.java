package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

public class Intake {

  private static boolean isRunning;

  public static final TalonFX mMasterIntakeMotor =
      new TalonFX(Constants.CAN_IDS.MASTER_INTAKE_MOTOR, "CANivore");
  public static final TalonFX mFollowIntakeMotor =
      new TalonFX(Constants.CAN_IDS.FOLLOWER_INTAKE_MOTOR, "CANivore");    
  public static final TalonFX mLiftMotor =
      new TalonFX(Constants.CAN_IDS.INT_PIVOT_MOTOR, "CANivore");
  public static final TalonFX mIndexMotor = 
      new TalonFX(Constants.CAN_IDS.INDEX_MOTOR);
  public static final TalonFX mFeedMotor = 
      new TalonFX(Constants.CAN_IDS.FEED_MOTOR);


  private static final MotionMagicVoltage mLiftMotorControlRequest = 
    new MotionMagicVoltage(0).withEnableFOC(true);

  private static final double STOW_POS = 0.0;
  private static final double DOWN_POS = 0.0;

//when you want to make a follow motor, you need to set the master motor to follow the follower motor. This is done by calling the setControl method on the master motor and passing in a Follower object with the follower motor's device ID and a boolean indicating whether the follower motor should be inverted or not.
  public static void init() {
    isRunning = false;
    
    TalonFXConfiguration masterIntakeMotorConfiguration = new TalonFXConfiguration();
    masterIntakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    masterIntakeMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    masterIntakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    masterIntakeMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    masterIntakeMotorConfiguration.CurrentLimits.StatorCurrentLimit = 90;
    masterIntakeMotorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 17;
    masterIntakeMotorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;

    mMasterIntakeMotor.getConfigurator().apply(masterIntakeMotorConfiguration);

    mFollowIntakeMotor.getConfigurator().apply(new TalonFXConfiguration());
    mFollowIntakeMotor.setControl(new Follower(Constants.CAN_IDS.MASTER_INTAKE_MOTOR, MotorAlignmentValue.Opposed));

    TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration();
    indexMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    mIndexMotor.getConfigurator().apply(indexMotorConfiguration);

     TalonFXConfiguration feedMotorConfiguration = new TalonFXConfiguration();
    indexMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    mFeedMotor.getConfigurator().apply(feedMotorConfiguration);

    TalonFXConfiguration liftMotorConfiguration = new TalonFXConfiguration();
    
    liftMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // These two line are saying that there is no current limit on the motor, and that the motor should break when it is not being powered.
    liftMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    liftMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
      // the limit that the moto cannot pass / setting the bounds where you motor can stay in; ised for intake motor.
    liftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // We want this lift motor in break so it holds it’s position when it is not being powered.
    liftMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    liftMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
    liftMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    liftMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    
    liftMotorConfiguration.Slot0.GravityType = GravityTypeValue.Arm_Cosine; 
// 
    // PID PIDF values for the motor controller. This is the feed forward and feedback for the motor controller PID loop.
    liftMotorConfiguration.Slot0.kG = 0.15;
    liftMotorConfiguration.Slot0.kS = 0.4;
    liftMotorConfiguration.Slot0.kV = 2.3;
    liftMotorConfiguration.Slot0.kA = 0.05;

    liftMotorConfiguration.Slot0.kP = 3;
    liftMotorConfiguration.Slot0.kI = 0.0;
    liftMotorConfiguration.Slot0.kD = 0.0;

    liftMotorConfiguration.Feedback.RotorToSensorRatio = 1;
    liftMotorConfiguration.Feedback.SensorToMechanismRatio = 22;

    liftMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2;
    liftMotorConfiguration.MotionMagic.MotionMagicAcceleration = 7;

    liftMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
    
    mLiftMotor.getConfigurator().apply(liftMotorConfiguration);
    
    mLiftMotor.setPosition(
      mLiftMotor.getPosition().waitForUpdate(0.02).
      getValue());

      mLiftMotor.getPosition().setUpdateFrequency(50.0);
  }

  public static void set(double output) {
    mMasterIntakeMotor.set(output);
  }

  public static void intake() {
    set(-0.85);
  }

  public static void outtake() {
    set(0.5);
  }

  public static void setAngle(double target) {
    mLiftMotor.setControl(mLiftMotorControlRequest.withPosition(target));
  }

  public static void intakeUp() {
    setAngle(STOW_POS);
  }

  public static void intakeDown() {
    setAngle(DOWN_POS);
  }

  public static void intakePivotStop() {
    mLiftMotor.set(0.0);
  }


  public static void setIndex(double speed) {
    mIndexMotor.set(speed);
  }

  public static void IndexIn() {
    setIndex(0.8);
  }

  public static void IndexOut() {
    setIndex(-0.7);
  }

    public static void setFeed(double speed) {
    mFeedMotor.set(speed);

    if (speed != 0) {
      isRunning = true;
    } else {
      isRunning = false;
    }
  }

  public static void FeedIn() {
    setFeed(0.8);
  }

  public static void FeedOut() {
    setFeed(-0.7);
  }

  public static void simulateIntakeUp() {
    Logger.recordOutput("Intake Component 1", new Pose3d(0.1225, 0, -0.0825, new Rotation3d(0, -30 * Math.PI / 180, 0)));
    Logger.recordOutput("Intake Component 2", new Pose3d(-0.3675, 0, 0.5025, new Rotation3d(0, 40 * Math.PI / 180, 0)));
  }

  public static void simulateIntakeDown() {
    Logger.recordOutput("Intake Component 1", new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    Logger.recordOutput("Intake Component 2", new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
  }
  public static void allRollersIn() {
    intake();
    IndexIn();
    FeedIn();
  }

  public static void allRollersOut() {
    outtake();
    IndexOut();
    FeedOut();
  }

  public static boolean isRunning() {
    return isRunning;
  }
}