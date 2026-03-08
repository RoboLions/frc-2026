package frc.robot.subsystems.interfaces;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Intake {

  private static final TalonFX mIntakeRollerMotor =
    new TalonFX(Constants.CAN_IDS.INTAKE_ROLLER, "CANexternal");  
  private static final TalonFX mSpindexMotor = 
    new TalonFX(Constants.CAN_IDS.INDEX_MOTOR, "CANexternal");
  private static final TalonFX mFeedMotor = 
    new TalonFX(Constants.CAN_IDS.FEED_MOTOR, "CANexternal");
  private static final TalonFX mRackMotor = 
    new TalonFX(Constants.CAN_IDS.RACK_MOTOR, "CANexternal");

  private static final double STOW_POS = 2.0;
  private static final double MIDDLE_POS = 0.0;
  private static final double DOWN_POS = 48.0;

  public static void init() {    
    TalonFXConfiguration masterIntakeMotorConfiguration = new TalonFXConfiguration();

    masterIntakeMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    masterIntakeMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    masterIntakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mIntakeRollerMotor.getConfigurator().apply(masterIntakeMotorConfiguration);

    TalonFXConfiguration indexMotorConfiguration = new TalonFXConfiguration();

    indexMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    indexMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    indexMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

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
    rackMotorConfiguration.CurrentLimits.SupplyCurrentLimit = 70;

    rackMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rackMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rackMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 48.5;
    rackMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rackMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.25;
    
    rackMotorConfiguration.Slot0.kP = 0.5;
    rackMotorConfiguration.Slot0.kI = 0.0;
    rackMotorConfiguration.Slot0.kD = 0.0;
    rackMotorConfiguration.Slot0.kS = 0.8;
    rackMotorConfiguration.Slot0.kV = 0.105;
    rackMotorConfiguration.Slot0.kA = 0.0;
    rackMotorConfiguration.Slot0.kG = 0.0;

    rackMotorConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
    rackMotorConfiguration.MotionMagic.MotionMagicAcceleration = 100.0;
    rackMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 75.0;
    
    mRackMotor.getConfigurator().apply(rackMotorConfiguration);
  }

  public static void set(double outputVoltage) {
    mIntakeRollerMotor.setControl(new VoltageOut(outputVoltage).withEnableFOC(true));
  }

  public static void intake() {
    set(4);
  }

  public static void outtake() {
    set(-4);
  }

  public static void setRack(double target) {
    mRackMotor.setControl(new MotionMagicVoltage(target).withEnableFOC(true));
  }

  public static void intakeUp() {
    setRack(STOW_POS);
  }

  public static void intakeDown() {
    setRack(DOWN_POS);
  }

  public static void intakePivotStop() {
    mRackMotor.set(0.0);
  }

  public static void setIndex(double voltageOut) {
    mSpindexMotor.setControl(new VoltageOut(voltageOut).withEnableFOC(true));
  }

  public static void IndexIn() {
    setIndex(10);
  }

  public static void IndexOut() {
    setIndex(-4);
  }

  public static void setFeed(double rpm) {
    mFeedMotor.setControl(new MotionMagicVelocityVoltage(rpm)
              .withUpdateFreqHz(100)
              .withEnableFOC(true));
  }

  public static void FeedIn() {
    setFeed(50);
  }

  public static void FeedOut() {
    setFeed(-10);
  }

  public static void stopFeed() {
    mFeedMotor.setControl(new VoltageOut(0).withEnableFOC(true));
  }

  // public static void simulateIntakeUp() {
  //   Logger.recordOutput("Intake Sim/ Intake Component 1", new Pose3d(0.1225, 0, -0.0825, new Rotation3d(0, -30 * Math.PI / 180, 0)));
  //   Logger.recordOutput("Intake Sim/ Intake Component 2", new Pose3d(-0.3675, 0, 0.5025, new Rotation3d(0, 40 * Math.PI / 180, 0)));
  // }

  // public static void simulateIntakeDown() {
  //   Logger.recordOutput("Intake Sim/ Intake Component 1", new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
  //   Logger.recordOutput("Intake Sim/ Intake Component 2", new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0)));
  // }

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
}