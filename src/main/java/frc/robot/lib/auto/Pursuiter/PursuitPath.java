package frc.robot.lib.auto.Pursuiter;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.lib.auto.Pursuiter.helpers.FastMath;
import frc.robot.lib.auto.Pursuiter.helpers.PathLoader;
import frc.robot.lib.auto.Pursuiter.helpers.PoseTolerance;
import frc.robot.lib.auto.Pursuiter.util.PursuitEventMarker;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;
import frc.robot.lib.auto.Pursuiter.util.PointConstraints;

public class PursuitPath {
    private final PoseTolerance poseTolerance;
    private final Distance lookAhead;
    private List<PathPoint> pathPoints;
    private PIDController translationController = new PIDController(0.5, 0, 0);
    private PIDController endPointController = new PIDController(3.0, 0, 0);
    private PIDController headingController = new PIDController(1.0, 0, 0);
    private PathPoint currentPoint;
    private List<PathPoint> eventPoints;
    private PathPoint lookAheadPoint;
    private boolean isFinished = false;
    private final String trajectoryName;
    private PursuitPath nextPath;
    private boolean logToggle = true;
    
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
        this.eventPoints = this.getEventPoints();
    }

    public PursuitPath(
        double metersTolerance, 
        double degreesTolerance,
        Distance lookAheadDistance,
        PIDController translationController,
        PIDController endPointController,
        PIDController headingController,
        String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(metersTolerance), Degrees.of(degreesTolerance));
        this.lookAhead = lookAheadDistance;
        this.translationController = translationController;
        this.endPointController = endPointController;
        this.headingController = headingController;
        this.pathPoints = PathLoader.loadSample(trajectoryName);
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.trajectoryName = trajectoryName;
        this.eventPoints = this.getEventPoints();
    }

    public PursuitPath(PursuitProfile profile, String trajectoryName) 
    {
        this.poseTolerance = new PoseTolerance(
            Meters.of(profile.metersTolerance()), Degrees.of(profile.degreesTolerance()));
        this.lookAhead = profile.lookAheadDistance();
        this.translationController = profile.translationController();
        this.endPointController = profile.endPointController();
        this.headingController = profile.headingController();
        this.pathPoints = PathLoader.loadSample(trajectoryName);
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        this.trajectoryName = trajectoryName;
        this.eventPoints = this.getEventPoints();

        if (logToggle) {exampleLog();}
    }

    public void update(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> outputConsumer) {
        if (isFinished) {return;}

        Pose2d robotPose2d = poseSupplier.get();

        this.currentPoint = 
            FastMath.findClosestPoint(robotPose2d, pathPoints, currentPoint.pointIndex(), lookAheadPoint.pointIndex());
        List<PathPoint> remainingPoints = pathPoints.subList(currentPoint.pointIndex(), pathPoints.size());

        for (int i = 0; i < eventPoints.size(); i++) {
            PathPoint event = eventPoints.get(i);
            if (!event.eventMarker().hasTriggered() && currentPoint.pointIndex() >= event.pointIndex()) {
                PathPoint removed = this.eventPoints.remove(0);
                System.out.println("Scheduled Binded-Command " + removed.eventMarker().getName() + " on path: " + trajectoryName);
                CommandScheduler.getInstance().schedule(event.eventMarker().getCommand());
            }
        }

        this.lookAheadPoint = FastMath.closestPointWithThreshold((
            (float) lookAhead.magnitude()), 
            currentPoint, 
            remainingPoints);

        if (currentPoint.pointIndex() >= pathPoints.get(pathPoints.size() - 1).pointIndex() && 
            poseTolerance.inError(currentPoint, robotPose2d)) {
            this.isFinished = true;
            return;
        }
        
        headingController.reset();
        
        double vx = lookAheadPoint.constraints().vx();
        double vy = lookAheadPoint.constraints().vy();
        double velocity = Math.hypot(vx, vy);

        double dx = lookAheadPoint.point().getX() - robotPose2d.getX();
        double dy = lookAheadPoint.point().getY() - robotPose2d.getY();
        double dist = Math.hypot(dx, dy);

        Rotation2d heading = new Rotation2d(Math.atan2(dy, dx));

        double pidAdjust = (dist <= lookAhead.magnitude()) 
            ? Math.abs(endPointController.calculate(dist)) 
            : Math.abs(translationController.calculate(dist));

        double omega = headingController.calculate(
            robotPose2d.getRotation().getRadians(), 
            currentPoint.point().getRotation().getRadians()
        );

        double fx = (velocity + pidAdjust) * heading.getCos();
        double fy = (velocity + pidAdjust) * heading.getSin();

        outputConsumer.accept(new ChassisSpeeds(fx, fy, omega));

        if (logToggle) {exampleLog();}
    }

    public PathPoint getLookAhead() {
        return isFinished ? nextPath.getLookAhead() : this.lookAheadPoint;
    }

    public PathPoint getCurrentPoint() {
        return isFinished ? nextPath.getCurrentPoint() : this.currentPoint;
    }

    /** Returns a Pose2d Array visualizing the pathpoints with a certain density parameter.
     * 
     * @param density Input value to skip every x poses to save bandwith. 5 is usually a good starting point.
     */
    public Pose2d[] getVisualizedPath(int density) {
        if (this.isFinished) {return nextPath.getVisualizedPath(density);}
        List<Pose2d> poses = new ArrayList<Pose2d>();
        Pose2d[] poseArr = poses.toArray(new Pose2d[0]);
        for (int i = 0; i < pathPoints.size(); i += density) {
            poses.add(pathPoints.get(i).point());
        } 
        poseArr = poses.toArray(new Pose2d[0]);
        return poseArr;
    }

    public boolean isFinished() {
        return this.isFinished;
    }

    /** Append nextPath so that when isFinished() returns true,
     *  next Path is executed immediately after when this path
     *  is called to a Command.
     * 
     * @param nextPath
     * @return
     */
    public PursuitPath append(PursuitPath nextPath) {
        if (this.nextPath == null) {
            this.nextPath = nextPath;
        } else {
            this.nextPath.append(nextPath);
        } return nextPath;
    }

    public void resetPathState() {
        this.isFinished = false;
        if (this.pathPoints == null || pathPoints.isEmpty()) {return;}
        this.currentPoint = pathPoints.get(0);
        this.lookAheadPoint = pathPoints.get(0);
        this.eventPoints = this.getEventPoints();
    }

    /**
     * Compiles the entire linked sequence of PursuitPaths into a single executable WPILib Command.
     * 
     * @param poseSupplier Provides the current robot pose (e.g., driveSubsystem::getPose)
     * @param outputConsumer Consumes the calculated target speeds (e.g., driveSubsystem::driveFieldRelative)
     * @param requirements The drive subsystem tracking requirements
     */
    public Command toCommand(
        Supplier<Pose2d> poseSupplier, 
        Consumer<ChassisSpeeds> outputConsumer, 
        Subsystem... requirements) 
    {
        Command currentSegmentCommand = new FunctionalCommand(
            () -> this.resetPathState(),
            () -> this.update(poseSupplier, outputConsumer),
            interrupted -> outputConsumer.accept(new ChassisSpeeds()),
            () -> this.isFinished(),
            requirements
        );

        if (this.nextPath == null) {
            return currentSegmentCommand;
        }

        return currentSegmentCommand.andThen(
            this.nextPath.toCommand(poseSupplier, outputConsumer, requirements)
        );
    }

    /**
     * Compiles the entire linked sequence of PursuitPaths into a single executable WPILib Command
     * without explicit subsystem requirements bound internally.
     * 
     * @param poseSupplier Provides the current robot pose (e.g., driveSubsystem::getPose)
     * @param outputConsumer Consumes the calculated target speeds (e.g., driveSubsystem::driveFieldRelative)
     */
    public Command toCommand(Supplier<Pose2d> poseSupplier, Consumer<ChassisSpeeds> outputConsumer) {
        Command currentSegmentCommand = new FunctionalCommand(
            () -> this.resetPathState(),
            () -> this.update(poseSupplier, outputConsumer),
            interrupted -> outputConsumer.accept(new ChassisSpeeds()),
            () -> this.isFinished()
        );

        if (this.nextPath == null) {
            return currentSegmentCommand;
        }

        return currentSegmentCommand.andThen(
            this.nextPath.toCommand(poseSupplier, outputConsumer)
        );
    }

    public void exampleLog() {
        Logger.recordOutput("Pursuiter/ exampleLog/ current-trajectory", getVisualizedPath(7));
        Logger.recordOutput("Pursuiter/ exampleLog/ current-point", this.currentPoint);
        Logger.recordOutput("Pursuiter/ exampleLog/ current-lookahead", this.lookAheadPoint);
    }

    public String getName() {
        return this.trajectoryName;
    }

    public List<PursuitEventMarker> getEventMarkers() {
        List<PursuitEventMarker> eventMarkers = new ArrayList<PursuitEventMarker>();
        for (PathPoint pt : pathPoints) {
            if (pt.eventMarker() != null) {
                eventMarkers.add(pt.eventMarker());
            }
        }   return eventMarkers;
    }

    public List<PathPoint> getEventPoints() {
        List<PathPoint> pts = new ArrayList<PathPoint>();
        for (PathPoint pt : pathPoints) {
            if (pt.eventMarker() != null) {
                pts.add(pt);
            }
        }   return pts;
    }

    public Pose2d[] getEventPoses() {
        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (PathPoint pt : pathPoints) {
            if (pt.eventMarker() != null) {
                poses.add(pt.point());
            }
        }   
        Pose2d[] poseArr = poses.toArray(new Pose2d[0]);
        return poseArr;
    }

    public void bindCommand(String commandName, Command command) {
        for (int i = 0; i < eventPoints.size(); i++) {
            if (commandName.equals(eventPoints.get(i).eventMarker().getName())) {
                eventPoints.get(i).eventMarker().bindCommand(command);
            }
        }
    }
    
    public void flipPath(boolean flipX, boolean flipY, Translation2d fieldCenter) {
        if (this.pathPoints.isEmpty() || this.pathPoints == null) {return;}
        List<PathPoint> flippedPoints = new ArrayList<>(); 

        for (PathPoint point : pathPoints) {
            Pose2d pose = point.point();

            double newX = flipX ? (2 * fieldCenter.getX()) - pose.getX(): pose.getX();
            double newY = flipY ? (2 * fieldCenter.getY()) - pose.getY(): pose.getY();
            double newvX = flipX ? -point.constraints().vx() : point.constraints().vx();
            double newvY = flipX ? -point.constraints().vy() : point.constraints().vy();
            double newaX = flipX ? -point.constraints().ax() : point.constraints().ax();
            double newaY = flipX ? -point.constraints().ay() : point.constraints().ay();
            
            double newRadians = pose.getRotation().getRadians();
            if (flipX && flipY) {
                newRadians += Math.PI;
            } else if (flipX) {
                newRadians = Math.PI - newRadians;
            } else if (flipY) {
                newRadians = -newRadians;
            }
            
            Rotation2d newRotation = new Rotation2d(newRadians);
            Pose2d newPose = new Pose2d(newX, newY, newRotation);

            boolean invertsRotation = flipX ^ flipY; 
            double newOmega = 
                invertsRotation ? -point.constraints().omega().magnitude() : point.constraints().omega().magnitude();

            PathPoint newPt = new PathPoint(
                newPose, 
                new PointConstraints(newvX, newvY, newaX, newaY, RadiansPerSecond.of(newOmega)), 
                point.pointIndex(), 
                point.eventMarker(), 
                point.timeStamp());

            flippedPoints.add(newPt);
        }   
        
        this.pathPoints = flippedPoints;
        this.resetPathState();

        if (this.nextPath != null) {
            this.nextPath.flipPath(flipX, flipY, fieldCenter);
        }
    }
}
