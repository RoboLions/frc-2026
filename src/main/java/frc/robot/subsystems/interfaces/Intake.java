package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Constants;

public class Intake {

  private static final TalonFX mIntakeRollerMotor =
    new TalonFX(Constants.CAN_IDS.INTAKE_ROLLER);  
  private static final TalonFX mIndexMotor = 
    new TalonFX(Constants.CAN_IDS.INDEX_MOTOR);
  private static final TalonFX mFeedMotor = 
    new TalonFX(Constants.CAN_IDS.FEED_MOTOR);

  private static final double STOW_POS = 0.0;
  private static final double DOWN_POS = 0.0;

  public static void init() {    
    TalonFXConfiguration masterIntakeMotorConfiguration = new TalonFXConfiguration();
    
    masterIntakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // TODO: MAKE THE REST OF THE CONFIGS FOR INTAKE ROLLER

    TalonFXConfiguration indexMotorCongirConfiguration = new TalonFXConfiguration();

    indexMotorCongirConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    indexMotorCongirConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    
    indexMotorCongirConfiguration.MotionMagic.MotionMagicAcceleration = 1;
    indexMotorCongirConfiguration.Feedback.SensorToMechanismRatio = 1 / 1;

    indexMotorCongirConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    TalonFXConfiguration feedMotorConfiguration = new TalonFXConfiguration();

    feedMotorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    feedMotorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    feedMotorConfiguration.Slot0.kA = 0.018;
    feedMotorConfiguration.Slot0.kV = 0.115;
    feedMotorConfiguration.Slot0.kS = 0.4;
    feedMotorConfiguration.Slot0.kP = 0.2;

    feedMotorConfiguration.MotionMagic.MotionMagicAcceleration = 125;
    feedMotorConfiguration.Feedback.SensorToMechanismRatio = 1 / 1;

    feedMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    mIntakeRollerMotor.getConfigurator().apply(masterIntakeMotorConfiguration);
    mIndexMotor.getConfigurator().apply(indexMotorCongirConfiguration);
    mFeedMotor.getConfigurator().apply(feedMotorConfiguration);
  }

  public static void set(double outputVoltage) {
    mIntakeRollerMotor.setControl(new VoltageOut(outputVoltage));
  }

  public static void intake() {
    set(5);
  }

  public static void outtake() {
    set(-5);
  }

  public static void setAngle(double target) {
    // mLiftMotor.setControl(mLiftMotorControlRequest.withPosition(target));
  }

  public static void intakeUp() {
    setAngle(STOW_POS);
  }

  public static void intakeDown() {
    setAngle(DOWN_POS);
  }

  public static void intakePivotStop() {
    // mLiftMotor.set(0.0);
  }

  public static void setIndex(double voltageOut) {
    mIndexMotor.setControl(new VoltageOut(voltageOut).withEnableFOC(true));
  }

  public static void IndexIn() {
    setIndex(9);
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
    setFeed(90);
  }

  public static void FeedOut() {
    setFeed(-10);
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
    // intake();
    IndexIn();
    FeedIn();
  }

  public static void allRollersOut() {
    // outtake();
    IndexOut();
    FeedOut();
  }
}