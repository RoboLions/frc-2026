package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Intake {

  private static final TalonFX mIntakeRollerMotor =
    new TalonFX(Constants.CAN_IDS.INTAKE_ROLLER, "CANexternal");  
  private static final TalonFX mIntakeRollerMotorFollow = 
    new TalonFX(Constants.CAN_IDS.INTAKE_FOLLOWER_ROLLER, "CANexternal");
  private static final TalonFX mSpindexMotor = 
    new TalonFX(Constants.CAN_IDS.INDEX_MOTOR, "CANexternal");
  private static final TalonFX mFeedMotor = 
    new TalonFX(Constants.CAN_IDS.FEED_MOTOR, "CANexternal");
  private static final TalonFX mRackMotor = 
    new TalonFX(Constants.CAN_IDS.RACK_MOTOR, "CANexternal");

  private static final double STOW_POS = 0.25;
  private static final double MIDDLE_POS = 26.9;
  private static final double DOWN_POS = 48.4;

  public static void init() {    
    TalonFXConfiguration masterIntakeMotorConfiguration = new TalonFXConfiguration();

    masterIntakeMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    masterIntakeMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    masterIntakeMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    masterIntakeMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    masterIntakeMotorConfiguration.CurrentLimits.StatorCurrentLimit = 120;

    masterIntakeMotorConfiguration.Slot0.kP = 3.0;
    masterIntakeMotorConfiguration.Slot0.kS = 3.25;
    masterIntakeMotorConfiguration.Slot0.kV = 0.001;
    masterIntakeMotorConfiguration.Slot0.kA = 0.0;

    masterIntakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mIntakeRollerMotorFollow.getConfigurator().apply(masterIntakeMotorConfiguration);
    mIntakeRollerMotor.getConfigurator().apply(masterIntakeMotorConfiguration);

    TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration();

    indexMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    indexMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    indexMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    indexMotorConfiguration.CurrentLimits.StatorCurrentLimit = 50;

    indexMotorConfiguration.Slot0.kP = 0.2;
    indexMotorConfiguration.Slot0.kS = 0.375;
    indexMotorConfiguration.Slot0.kV = 0.094;
    indexMotorConfiguration.Slot0.kA = 0.001;

    indexMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mSpindexMotor.getConfigurator().apply(indexMotorConfiguration);

    TalonFXConfiguration feedMotorConfiguration = new TalonFXConfiguration();

    feedMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    feedMotorConfiguration.Slot0.kP = 0.5;
    feedMotorConfiguration.Slot0.kI = 0.0;
    feedMotorConfiguration.Slot0.kD = 0.0;
    feedMotorConfiguration.Slot0.kS = 0.37;
    feedMotorConfiguration.Slot0.kA = 0.003;
    feedMotorConfiguration.Slot0.kV = 0.1185;
    feedMotorConfiguration.Slot0.kG = 0.0;

    feedMotorConfiguration.MotionMagic.MotionMagicAcceleration = 250;
    feedMotorConfiguration.Feedback.SensorToMechanismRatio = 1 / 1;
    feedMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mFeedMotor.getConfigurator().apply(feedMotorConfiguration);

    TalonFXConfiguration rackMotorConfiguration = new TalonFXConfiguration();
    
    rackMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rackMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    rackMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 60;
    rackMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    rackMotorConfiguration.CurrentLimits.StatorCurrentLimit = 60;

    rackMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rackMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rackMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = DOWN_POS;
    rackMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rackMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = STOW_POS;
    
    rackMotorConfiguration.Slot0.kP = 0.5;
    rackMotorConfiguration.Slot0.kI = 0.0;
    rackMotorConfiguration.Slot0.kD = 0.0;
    rackMotorConfiguration.Slot0.kS = 0.8;
    rackMotorConfiguration.Slot0.kV = 0.105;
    rackMotorConfiguration.Slot0.kA = 0.0;
    rackMotorConfiguration.Slot0.kG = 0.0;

    rackMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
    rackMotorConfiguration.MotionMagic.MotionMagicAcceleration = 200.0;
    rackMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 300.0;
    
    mRackMotor.getConfigurator().apply(rackMotorConfiguration);
    // mRackMotor.setPosition(0);
  }

  public static void set(double speed) {
    mIntakeRollerMotor.setControl(new VelocityTorqueCurrentFOC(speed).withUpdateFreqHz(20));
    mIntakeRollerMotorFollow.setControl(new Follower(mIntakeRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(20));
  }

  public static void intake() {
    set(70);
  }

  public static void intakeFastAuto() {
    set(85);
  }

  public static void outtake() {
    set(-50);
  }

  public static void stopIntake() {
    mIntakeRollerMotor.setControl(new VoltageOut(0).withUpdateFreqHz(20));
    mIntakeRollerMotorFollow.setControl(new Follower(mIntakeRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed).withUpdateFreqHz(20));
  }

  public static void setRack(double target) {
    mRackMotor.setControl(new MotionMagicVoltage(target).withEnableFOC(true).withUpdateFreqHz(20));
  }

  public static void intakeUp() {
    setRack(STOW_POS);
  }

  public static void intakeMid() {
    setRack(MIDDLE_POS);
  }

  public static void intakeDown() {
    setRack(DOWN_POS);
  }

  public static double getRackPosition() {
    return mRackMotor.getPosition().getValueAsDouble();
  }

  public static void setIndex(double velocity) {
    mSpindexMotor.setControl(new VelocityVoltage(velocity).withEnableFOC(true).withUpdateFreqHz(20));
  }

  public static void IndexIn() {
    setIndex(95);
  }

  public static void IndexOut() {
    setIndex(-50);
  }

  public static void stopIndex() {
   mSpindexMotor.setControl(new VoltageOut(0)); 
  }

  public static void setFeed(double rpm) {
    mFeedMotor.setControl(new MotionMagicVelocityVoltage(rpm)
              .withUpdateFreqHz(20)
              .withEnableFOC(true));
  }

  public static void FeedIn() {
    setFeed(50);
  }

  public static void FeedOut() {
    setFeed(-10);
  }

  public static void stopFeed() {
    mFeedMotor.setControl(new VoltageOut(0));
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

  public static void allRollersStop() {
    stopIntake();
    stopIndex();
    stopFeed();
  }
}