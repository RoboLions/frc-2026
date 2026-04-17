package frc.robot.subsystems.interfaces.swerve;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoFactory;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotMap;

public class Swerve {

    public class SwerveConstants{
        public static final double ODOMETRY_FREQUENCY = 150.0;

        private static final double SLIP_ERROR_THRESHOLD = 1.0; //needs to be tuned against wall
        private static final double MaxSpeed = GeneratedConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    private class SwerveObjects{
        public static final CommandSwerveDrivetrain Swerve = 
            GeneratedConstants.createDrivetrain();

        private static SwerveDriveState lastReadState = new SwerveDriveState();
        private static double loopLatencySec;

        private static final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.MaxSpeed * 0.075).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.075) // Add a 7.5% deadband
                .withDriveRequestType(DriveRequestType.Velocity);

        private static final SwerveRequest.FieldCentric closedLoopDrive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.MaxSpeed * 0.075).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.075) // Add a 7.5% deadband
                .withDriveRequestType(DriveRequestType.Velocity);  

        private static final PIDController pointDriveController = new PIDController(1, 0, 0);
        private static final PIDController headingController = new PIDController(1.75, 0, 0.04);

        private static final SlewRateLimiter xLimiter = new SlewRateLimiter(1.0); // units/sec²
        private static final SlewRateLimiter yLimiter = new SlewRateLimiter(1.0);
        private static final SlewRateLimiter omegaLimiter = new SlewRateLimiter(4);

        private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    }

    private class TelemetryObjects{
        private static Matrix<N3, N1> odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);;

        private static final TalonFX mDRIVE0 = getModuleStates()[0].getDriveMotor();
        private static final TalonFX mDRIVE1 = getModuleStates()[1].getDriveMotor();
        private static final TalonFX mDRIVE2 = getModuleStates()[2].getDriveMotor();
        private static final TalonFX mDRIVE3 = getModuleStates()[3].getDriveMotor();

        private static final GeneratedTelemetry telemetryLogger = 
            new GeneratedTelemetry(SwerveConstants.MaxSpeed);
        
        private static StructPublisher<Pose3d> mechanismPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Drivetrain", Pose3d.struct)
			.publish();

        private static final Field2d elasticPose = 
            new Field2d();
    }
    
    public static void init() {
        SwerveObjects.headingController.enableContinuousInput(-Math.PI, Math.PI);
        SwerveObjects.pointDriveController.setTolerance(0.001);

        SwerveObjects.Swerve.registerTelemetry(TelemetryObjects.telemetryLogger::telemeterize);
    }

    public static void periodic() {
        setModuleDeviations(); // this is actually one the first methods to be run

        Logger.recordOutput("Swerve/ 2D CTRE Pose-Estimate", getPose());
        Logger.recordOutput("Swerve/ FieldSpeeds", getFieldSpeeds());
    }

    public static void disabledPeriodic() {
        SwerveObjects.Swerve.periodic(); // look at the function comment and see that this is actually just a reorientation tool
    }

    public static void simulationPeriodic() {
        SwerveObjects.Swerve.updateSimState(0.005, RobotController.getBatteryVoltage());
        SwerveObjects.Swerve.simulationPeriodic();  
    }

    public static AutoFactory createAutoFactory() {
        return SwerveObjects.Swerve.createAutoFactory();
    }

    public static CommandSwerveDrivetrain getGeneratedDrive() {
		return SwerveObjects.Swerve;
	}

    public static void setModuleDeviations() { // TODO: figure out what to do with this method
        //all velocities are in RPS
        double mod0Velocity = Math.abs(TelemetryObjects.mDRIVE0.getVelocity().getValueAsDouble());
        double mod0Error = Math.abs(TelemetryObjects.mDRIVE0.getClosedLoopError().getValueAsDouble());

        double mod1Velocity = Math.abs(TelemetryObjects.mDRIVE1.getVelocity().getValueAsDouble());
        double mod1Error = Math.abs(TelemetryObjects.mDRIVE1.getClosedLoopError().getValueAsDouble());
        
        double mod2Velocity = Math.abs(TelemetryObjects.mDRIVE2.getVelocity().getValueAsDouble());
        double mod2Error = Math.abs(TelemetryObjects.mDRIVE2.getClosedLoopError().getValueAsDouble());

        double mod3Velocity = Math.abs(TelemetryObjects.mDRIVE3.getVelocity().getValueAsDouble());
        double mod3Error = Math.abs(TelemetryObjects.mDRIVE3.getClosedLoopError().getValueAsDouble());

        double maxError = Math.max(
            Math.max(mod0Error, mod1Error), 
            Math.max(mod2Error, mod3Error)
        );
        Logger.recordOutput("Swerve/ Velocity-Error/ MaxError", maxError);

        if (maxError > SwerveConstants.SLIP_ERROR_THRESHOLD) {
            TelemetryObjects.odometryMatrix = VecBuilder.fill(1.5 + maxError, 1.5 + maxError, 0.1);
        } else {
            TelemetryObjects.odometryMatrix = VecBuilder.fill(0.1, 0.1, 0.1);
        }

        SwerveObjects.Swerve.setStateStdDevs(TelemetryObjects.odometryMatrix);
        
        Logger.recordOutput("Swerve/ Motor-Velocities/ Mod0", mod0Velocity);
        Logger.recordOutput("Swerve/ Motor-Velocities/ Mod1", mod1Velocity);
        Logger.recordOutput("Swerve/ Motor-Velocities/ Mod2", mod2Velocity);
        Logger.recordOutput("Swerve/ Motor-Velocities/ Mod3", mod3Velocity);

        Logger.recordOutput("Swerve/ Velocity-Error/ Mod0", mod0Error);
        Logger.recordOutput("Swerve/ Velocity-Error/ Mod1", mod1Error);
        Logger.recordOutput("Swerve/ Velocity-Error/ Mod2", mod2Error);
        Logger.recordOutput("Swerve/ Velocity-Error/ Mod3", mod3Error);
    }

    public static SwerveModule<TalonFX, TalonFX, CANcoder>[] getModuleStates() {
        return getGeneratedDrive().getModules();
    }

    public static SwerveDriveState getState() {
		return SwerveObjects.Swerve.getState();
	}

	public static Pose2d getPose() {
		return getState().Pose;
	}

    public static double getYawAsDegrees() {
        return Swerve.getPose().getRotation().getDegrees();
    }

    public static Rotation2d getYawAsRotations() {
        return Swerve.getPose().getRotation();
    }

    public static double getYawAsRadians() {
        return Swerve.getPose().getRotation().getRadians();
    }

    public static double getYawRateAsRad() {
        return Swerve.getState().Speeds.omegaRadiansPerSecond;
    }

    public static double getYawRateAsDeg() {
        return Math.toDegrees(Swerve.getState().Speeds.omegaRadiansPerSecond);
    }

    public static double getLoopLatencySec() {
        return SwerveObjects.loopLatencySec;
    }

    public static Pose3d getPose3d() {
        Translation3d translation = new Translation3d(getPose().getX(), getPose().getY(), 0);
        Rotation3d rotation = new Rotation3d(getState().Pose.getRotation());
        return new Pose3d(translation, rotation);
    }

    public static double getDistToPoseAsDouble(Pose2d pose) {
        return getPose().getTranslation().getDistance(pose.getTranslation());
    }

	public static void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		SwerveObjects.Swerve.addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

	public static void resetPose(Pose2d pose) {
		SwerveObjects.Swerve.resetPose(pose);
	}

    public static void resetOdometry() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            resetPose(
                new Pose2d(
                    getPose().getTranslation(), 
                    new Rotation2d(0))); 
        } else {
            resetPose(
                new Pose2d(
                    getPose().getTranslation(), 
                    Rotation2d.fromDegrees(180))); 
        }
    }

    public static void zeroPose() {
        Pose2d pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
		SwerveObjects.Swerve.resetPose(pose);
	}

    public static void zeroGyro() {
        Pose2d pose = Swerve.getPose();
		SwerveObjects.Swerve.resetPose(new Pose2d(pose.getTranslation(), new Rotation2d(Math.PI)));
	}

    public static void zeroCommand() {
        SwerveObjects.Swerve.setControl(SwerveObjects.teleopDrive
                                        .withVelocityX(0)
                                        .withVelocityY(0)
                                        .withRotationalRate(0));
    }

    /**
     * @return Absolute value positive Meters per second of robot-centric speed.
     */
    public static double chassisMPSX() {
         ChassisSpeeds ChassisSpeeds = SwerveObjects.Swerve.getState().Speeds;
            return ChassisSpeeds.vxMetersPerSecond;
    }

    public static double chassisMPSY() {
        ChassisSpeeds ChassisSpeeds = SwerveObjects.Swerve.getState().Speeds;
           return ChassisSpeeds.vyMetersPerSecond;
   }

    public static ChassisSpeeds getFieldSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getState().Speeds, getYawAsRotations());
    }
    
    public static void brakeX() {
        SwerveObjects.Swerve.setControl(SwerveObjects.brake);
    }

    public static void teleopDrive() {
        double vy = -RobotMap.driverController.getLeftX();
        double vx = -RobotMap.driverController.getLeftY();
        double omega = -RobotMap.driverController.getRightX();

        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive.withVelocityX(vx * SwerveConstants.MaxSpeed)
                .withVelocityY(vy * SwerveConstants.MaxSpeed)
                .withRotationalRate(omega * SwerveConstants.MaxAngularRate));
    }

    public static void teleopDrive(double percentSpeed) {
        double vy = -RobotMap.driverController.getLeftX() * percentSpeed;
        double vx = -RobotMap.driverController.getLeftY() * percentSpeed;
        double omega = -RobotMap.driverController.getRightX() * percentSpeed;

        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive.withVelocityX(vx * SwerveConstants.MaxSpeed)
                .withVelocityY(vy * SwerveConstants.MaxSpeed)
                .withRotationalRate(omega * SwerveConstants.MaxAngularRate));
    }

    public static void teleopDriveSlewed(double percentSpeed) {
        double vy = -RobotMap.driverController.getLeftX() * percentSpeed;
        double vx = -RobotMap.driverController.getLeftY() * percentSpeed;
        double omega = -RobotMap.driverController.getRightX() * percentSpeed;

        double vxSlewed = SwerveObjects.xLimiter.calculate(vx);
        double vySlewed = SwerveObjects.yLimiter.calculate(vy);
        double omegaSlewed = SwerveObjects.omegaLimiter.calculate(omega);

        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive
                .withVelocityX(vxSlewed * SwerveConstants.MaxSpeed)
                .withVelocityY(vySlewed * SwerveConstants.MaxSpeed)
                .withRotationalRate(omegaSlewed * SwerveConstants.MaxAngularRate));
    }

    public static void maxVoltForward() {
        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive.withVelocityX(0)
                .withVelocityY(GeneratedConstants.kSpeedAt12Volts)
                .withRotationalRate(0));
    }

    private static void automaticDrive(double velocity, Rotation2d heading, double omega) {
        double vx = velocity * heading.getCos();
        double vy = velocity * heading.getSin();

        SwerveObjects.Swerve.setControl(
            SwerveObjects.closedLoopDrive.withVelocityX(vx * SwerveConstants.MaxSpeed)
                .withVelocityY(vy * SwerveConstants.MaxSpeed)
                .withRotationalRate(omega * SwerveConstants.MaxAngularRate));
    }

    public static void driveToPoint(Pose2d targetPose, double percentMaxSpeed) {
        Pose2d currPose = getPose();
        double dy = targetPose.getY() - currPose.getY();
        double dx = targetPose.getX() - currPose.getX();
        double distance = targetPose.getTranslation().getDistance(currPose.getTranslation());

        SwerveObjects.pointDriveController.setSetpoint(distance);
        double velocity = Math.min(SwerveObjects.pointDriveController.calculate(0), percentMaxSpeed);

        SwerveObjects.headingController.setSetpoint(targetPose.getRotation().getRadians());
        double omega = SwerveObjects.headingController.calculate(currPose.getRotation().getRadians());

        automaticDrive(velocity, new Rotation2d(Math.atan2(dy, dx)), omega);
    }

    public static void facePose(Translation2d targetPose, Rotation2d offset) {
        Pose2d currPose = getPose();
        double dy = targetPose.getY() - currPose.getY();
        double dx = targetPose.getX() - currPose.getX();
        Rotation2d target = new Rotation2d(Math.atan2(dy, dx));

        SwerveObjects.headingController.setSetpoint(target.getRadians() + offset.getRadians());
        double omega = SwerveObjects.headingController.calculate(currPose.getRotation().getRadians());

        automaticDrive(0, new Rotation2d(0), omega);
    }

    public static Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return SwerveObjects.Swerve.sysIdQuasistatic(direction);
    }

    public static Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return SwerveObjects.Swerve.sysIdDynamic(direction);
    }

    public void outputTelemetry() {
		TelemetryObjects.mechanismPublisher.set(new Pose3d(getPose()));
		TelemetryObjects.telemetryLogger.telemeterize(SwerveObjects.lastReadState);
		TelemetryObjects.elasticPose.setRobotPose(getPose());
		SmartDashboard.putData("Elastic Field 2D", TelemetryObjects.elasticPose);
	}

    public static void addLimelightMeasurement(Pose2d inputPose, double timestampSeconds, Vector<N3> standardDeviations) {
        SwerveObjects.Swerve.addVisionMeasurement(inputPose, timestampSeconds, standardDeviations);
    }
}
