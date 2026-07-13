package frc.robot.lib.auto.Pursuiter;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.robot.lib.auto.Pursuiter.helpers.FastMath;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;
import frc.robot.lib.auto.Pursuiter.helpers.PoseTolerance;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;

public class PursuitPath {
    private final PoseTolerance poseTolerance;
    private final Distance lookAhead;
    private List<PathPoint> pathPoints;
    private PIDController translationController = new PIDController(0.5, 0, 0);
    private PIDController headingController = new PIDController(1.0, 0, 0);
    private PathPoint currentPoint;
    private PathPoint lookAheadPoint;
    private boolean isFinished = false;
    private final String trajectoryName;
    
    /**
     * 
     * @param metersTolerance
     * @param degreesTolerance
     * @param lookAheadDistance
     * @param trajectoryName Should include the .traj suffix. Ex: "leave.traj" for a Choreo path called leave.
    */
    public PursuitPath(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.trajectoryName = trajectoryName;
    }

    public PursuitPath(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        PIDController translationController,
        PIDController headingController,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.translationController = translationController;
        this.headingController = headingController;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.trajectoryName = trajectoryName;
    }

    public ChassisSpeeds update(Supplier<Pose2d> poseSupplier) {
        if (isFinished) {
            System.out.println("Finished traj, tried to repeat!");
            return new ChassisSpeeds();
        }

        Pose2d robotPose2d = poseSupplier.get();

        this.currentPoint = 
            FastMath.findClosestPoint(robotPose2d, pathPoints, currentPoint.pointIndex(), lookAheadPoint.pointIndex());
        List<PathPoint> remainingPoints = pathPoints.subList(currentPoint.pointIndex(), pathPoints.size());

        this.lookAheadPoint = FastMath.closestPointWithThreshold((
            (float) lookAhead.magnitude()), 
            currentPoint, 
            remainingPoints);

        if (currentPoint.pointIndex() >= pathPoints.get(pathPoints.size() - 1).pointIndex() && 
            poseTolerance.inError(currentPoint, robotPose2d)) {
            this.isFinished = true;
            System.out.println("Finished Choreo, now marking done.");
            return new ChassisSpeeds();
        }
        
        headingController.reset();
        
        double vx = lookAheadPoint.constraints().vx();
        double vy = lookAheadPoint.constraints().vy();
        double velocity = Math.hypot(vx, vy);

        Logger.recordOutput("Pursuiter/ " + trajectoryName + "/Total Velocity Pursuit Component", velocity);

        double dx = lookAheadPoint.point().getX() - robotPose2d.getX();
        double dy = lookAheadPoint.point().getY() - robotPose2d.getY();
        double dist = Math.hypot(dx, dy);
        Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));

        double pidAdjust = Math.abs(translationController.calculate(dist));
        
        double omega = headingController.calculate(
            robotPose2d.getRotation().getRadians(), 
            currentPoint.point().getRotation().getRadians()
        );

        double fx = (velocity + pidAdjust) * heading.getCos();
        double fy = (velocity + pidAdjust) * heading.getSin();

        return new ChassisSpeeds(fx, fy, omega);
    }

    public PathPoint getLookAhead() {
        return this.lookAheadPoint;
    }

    public PathPoint getCurrentPoint() {
        return this.currentPoint;
    }
}
