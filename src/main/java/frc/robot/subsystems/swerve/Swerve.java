package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Swerve {

    public class SwerveConstants{
        public static final double ODOMETRY_FREQUENCY = 150.0;

        private static final double MaxSpeed = GeneratedConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    }

    private class SwerveObjects{

        public static final CommandSwerveDrivetrain Swerve = 
            GeneratedConstants.createDrivetrain();

        private static SwerveDriveState lastReadState;

        private static final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
                .withDeadband(SwerveConstants.MaxSpeed * 0.1).withRotationalDeadband(SwerveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
            
        private static SwerveRequest.ApplyFieldSpeeds m_ApplyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds();
        private static PIDController xController = new PIDController(0, 0, 0);
        private static PIDController yController = new PIDController(0, 0, 0);
        private static PIDController thetaController = new PIDController(0, 0, 0);
    }

    private class TelemetryObjects{
        private static final GeneratedTelemetry telemetryLogger = 
            new GeneratedTelemetry(SwerveConstants.MaxSpeed);
        
        private static StructPublisher<Pose3d> mechanismPublisher = NetworkTableInstance.getDefault()
			.getStructTopic("Mechanisms/Drivetrain", Pose3d.struct)
			.publish();

        private static final Field2d elasticPose = 
            new Field2d();
    }
    
    public static void init() {

    }

    public static void periodic() {
        SwerveObjects.Swerve.periodic(); // look at the function comment and see that this is actually just a reorientation tool
    }

    public static CommandSwerveDrivetrain getGeneratedDrive() {
		return SwerveObjects.Swerve;
	}

    public static SwerveDriveState getState() {
		return SwerveObjects.Swerve.getState();
	}

	public static Pose2d getPose() {
		return SwerveObjects.Swerve.getState().Pose;
	}

	public static void addVisionUpdate(Pose2d pose, Time timestamp, Matrix<N3, N1> stdDevs) {
		SwerveObjects.Swerve.addVisionMeasurement(pose, timestamp.in(Units.Seconds), stdDevs);
	}

	public static void resetPose(Pose2d pose) {
		SwerveObjects.Swerve.resetPose(pose);
	}

    public static void zeroPose() {
        Pose2d pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
		SwerveObjects.Swerve.resetPose(pose);
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

    public static double getDistToPose(Pose2d pose) {
        return 0.0; // TODO: do this
    }

    /**
     * Follows the given field-centric path sample with PID.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        SwerveObjects.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var pose = getState().Pose;

        var targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vxMetersPerSecond += SwerveObjects.xController.calculate(
            pose.getX(), sample.x
        );
        targetSpeeds.vyMetersPerSecond += SwerveObjects.yController.calculate(
            pose.getY(), sample.y
        );
        targetSpeeds.omegaRadiansPerSecond += SwerveObjects.thetaController.calculate(
            pose.getRotation().getRadians(), sample.heading
        );

        SwerveObjects.Swerve.setControl(
            SwerveObjects.m_ApplyFieldSpeeds.withSpeeds(targetSpeeds)
                .withWheelForceFeedforwardsX(sample.moduleForcesX())
                .withWheelForceFeedforwardsY(sample.moduleForcesY())
        );
    }

    public static void teleopDrive() {
        double vx = -RobotMap.driverController.getLeftX();
        double vy = -RobotMap.driverController.getLeftY();
        double omega = -RobotMap.driverController.getRightX();

        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive.withVelocityX(vx * SwerveConstants.MaxSpeed)
                .withVelocityY(vy * SwerveConstants.MaxSpeed)
                .withRotationalRate(omega * SwerveConstants.MaxAngularRate));

        Logger.recordOutput("In TeleopDrive?", true);

        SwerveObjects.Swerve.registerTelemetry(TelemetryObjects.telemetryLogger::telemeterize);

        Logger.recordOutput("CTRE Pose-Estimate", Swerve.getPose());
    }

    public static void automaticDrive(double velocity, Rotation2d heading) {
        double vx = velocity * heading.getCos();
        double vy = velocity * heading.getSin();
        double omega = 0;

        SwerveObjects.Swerve.setControl(
            SwerveObjects.teleopDrive.withVelocityX(vx * SwerveConstants.MaxSpeed)
                .withVelocityY(vy * SwerveConstants.MaxSpeed)
                .withRotationalRate(omega * SwerveConstants.MaxAngularRate));

        Logger.recordOutput("In AutomaticDrive?", true);

        SwerveObjects.Swerve.registerTelemetry(TelemetryObjects.telemetryLogger::telemeterize);

        Logger.recordOutput("CTRE Pose-Estimate", Swerve.getPose());
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
