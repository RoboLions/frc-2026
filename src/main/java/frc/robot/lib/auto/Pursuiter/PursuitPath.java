package frc.robot.lib.auto.Pursuiter;

import java.util.List;
import java.util.function.Supplier;

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
import frc.robot.subsystems.interfaces.swerve.Swerve;

public class PursuitPath {
    private final PoseTolerance poseTolerance;
    private final Distance lookAhead;
    private List<PathPoint> pathPoints;
    private PIDController xController = new PIDController(0.5, 0, 0);
    private PIDController yController = new PIDController(0.5, 0, 0);
    private PIDController headingController = new PIDController(1.0, 0, 0);
    private PathPoint currentPoint;
    private PathPoint lookAheadPoint;
    private boolean isFinished = false;
    
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
    }

    public PursuitPath(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        PIDController xController,
        PIDController yController,
        PIDController headingController,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.xController = xController;
        this.yController = yController;
        this.headingController = headingController;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public ChassisSpeeds update(Supplier<Pose2d> poseSupplier) {
        if (isFinished) {
            return new ChassisSpeeds();
        }

        Pose2d robotPose2d = poseSupplier.get();

        this.currentPoint = FastMath.findClosestPoint(robotPose2d, pathPoints);
        List<PathPoint> remainingPoints = pathPoints.subList(currentPoint.pointIndex(), pathPoints.size());

        this.lookAheadPoint = FastMath.closestPointWithThreshold((
            (float) lookAhead.magnitude()), 
            robotPose2d, 
            remainingPoints);

        if (currentPoint.pointIndex() >= pathPoints.get(pathPoints.size() - 1).pointIndex() && 
            poseTolerance.inError(currentPoint, robotPose2d)) {
            this.isFinished = true;
            return new ChassisSpeeds();
        }

        // if (!poseTolerance.inError(currentPoint, robotPose2d)) {
        //     recoveryAction();
        //     return new edu.wpi.first.math.kinematics.ChassisSpeeds(); 
        // }
        
        double vx = lookAheadPoint.constraints().vx();
        double vy = lookAheadPoint.constraints().vy();
        double velocity = Math.hypot(vx, vy);

        double dx = lookAheadPoint.point().getX() - robotPose2d.getX();
        double dy = lookAheadPoint.point().getY() - robotPose2d.getY();
        Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));
        
        double omega = headingController.calculate(
            robotPose2d.getRotation().getRadians(), 
            currentPoint.point().getRotation().getRadians()
        );

        double fx = velocity * heading.getCos() + xController.calculate(dx);
        double fy = velocity * heading.getSin() + yController.calculate(dy);

        return new ChassisSpeeds(fx, fy, omega);
    }

    public PathPoint getLookAhead() {
        return this.lookAheadPoint;
    }

    public PathPoint getCurrentPoint() {
        return this.currentPoint;
    }

    // private void recoveryAction() {}
}
