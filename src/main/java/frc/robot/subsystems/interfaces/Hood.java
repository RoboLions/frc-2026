package frc.robot.subsystems.interfaces;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class Hood {
    
    private static final TalonFX mFollowerPivotMotor =
      new TalonFX(Constants.CAN_IDS.BACK_FLYWHEEL_MOTOR);
    private static final TalonFX mMasterPivotMotor =
      new TalonFX(Constants.CAN_IDS.FRONT_MASTER_FLYWHEEL_MOTOR);  

    public static final StatusSignal<Angle> mBackMotorVelo = mFollowerPivotMotor.getPosition();
    public static final StatusSignal<Angle> mFrontMotorVelo = mMasterPivotMotor.getPosition();

    //PID for simulation hood
    private static final PIDController simulationTurretController = new PIDController(0.15, 0, 0.0, 0.025);
    private static double desiredTurretAngleFieldRel;
    private static double simulationTurretAngle;

    public static void init() {
        TalonFXConfiguration frontPivotConfig = new TalonFXConfiguration();

        frontPivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
        frontPivotConfig.CurrentLimits.SupplyCurrentLimit = 60;
        frontPivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        frontPivotConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        frontPivotConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        frontPivotConfig.TorqueCurrent.PeakForwardTorqueCurrent = 100;
        frontPivotConfig.TorqueCurrent.PeakReverseTorqueCurrent = -100;

        frontPivotConfig.Slot0.kS = 0;
        frontPivotConfig.Slot0.kV = 0;
        frontPivotConfig.Slot0.kA = 0;
        frontPivotConfig.Slot0.kP = 0;
        frontPivotConfig.Slot0.kI = 0;
        frontPivotConfig.Slot0.kD = 0;

        frontPivotConfig.MotionMagic.MotionMagicAcceleration = 1;

        frontPivotConfig.Feedback.SensorToMechanismRatio = 1 / 1;

        frontPivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        frontPivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        frontPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        frontPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        frontPivotConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 1;
        frontPivotConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -1;
        frontPivotConfig.ClosedLoopGeneral.ContinuousWrap = false;

        mMasterPivotMotor.getConfigurator().apply(frontPivotConfig);
        mFollowerPivotMotor.getConfigurator().apply(frontPivotConfig);
  }

  public static double calculateTurretAngle(Pose2d currPose, Pose2d targetPose, Translation2d currentVelocity) {
    double dx = targetPose.getX() - currPose.getX();
    double dy = targetPose.getY() - currPose.getY();

    return Math.atan2(dy, dx);
  }

  /** 
  * 
  * @param v The velocity as the piece exits the robot's shooter.
  * @param d The distance from the exit point to the target.
  * @param h The height from the exit point to the target.
  * @return angle in degrees that the robot should aim at.
  */
  public static double calculateHoodAngle(double v, double d, double h) {
    final double g = Constants.Hood.G;
    double v2 = v * v;
    double v4 = v * v * v * v;
    double d2 = d * d;
    
        double discriminant = v4 - g * (g * d2 + 2.0 * h * v2);
        if (discriminant < 0.0 || d == 0.0) {
            return Double.NaN; //No physical solution
        }
      double numerator = v2 + Math.sqrt(discriminant);
      double denominator = g * d;
      double angleRadians = Math.atan(numerator / denominator);
      double o = Math.toDegrees(angleRadians);
    return Constants.Hood.THETA_ANGLE_FROM_SHOOTER - o;
  } 

  // reminder that this is field relative, not robot relative like the actual turret must be.
  public static void simulateTurretAngle(Pose2d currPose, 
                                         Translation2d targetPose, 
                                         double robotFieldYaw,
                                         double robotYawRate) 
  {
    double dx = targetPose.getX() - currPose.getX();
    double dy = targetPose.getY() - currPose.getY();
    double r2 = dx * dx + dy * dy;
    double timeOFlight = Math.sqrt(r2) / Constants.Hood.FUEL_VELOCITY;

    double rotationalLead = robotYawRate * timeOFlight; //the amount of radians lead that the rotational component requires
    double totalLeadOffsets = -(rotationalLead);

    desiredTurretAngleFieldRel = Math.atan2(dy, dx) - robotFieldYaw + totalLeadOffsets;
    simulationTurretController.setSetpoint(desiredTurretAngleFieldRel);

    simulationTurretAngle += simulationTurretController.calculate(simulationTurretAngle);
    Logger.recordOutput("Turret 3D Pose", new Pose3d(0, 0, 0, new Rotation3d(0 , 0, simulationTurretAngle)));
  }
}