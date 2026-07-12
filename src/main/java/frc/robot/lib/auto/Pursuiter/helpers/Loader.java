package frc.robot.lib.auto.Pursuiter.helpers;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.auto.Pursuiter.util.PathPoint;
import frc.robot.lib.auto.Pursuiter.util.PointConstraints;

public class Loader {

    public static List<PathPoint> loadSample(String trajectoryName) {
        List<PathPoint> points = new ArrayList<>();
        ObjectMapper mapper = new ObjectMapper();
        
        try { 
            //for context Choreo generates a bunch of samples based on your path
            //we use those for the pure pursuit controller, not the points you make
            //pros: maintained choreo's benefits
            //cons: none this is goat.
            File traj = new File(Filesystem.getDeployDirectory(), "choreo/" + trajectoryName);

            JsonNode root = mapper.readTree(traj);
            JsonNode nodes = root.path("trajectory").path("samples");

            if (nodes.isMissingNode() || !nodes.isArray()) {
                DriverStation.reportError("Choreo file structure is invalid: " + traj.getName(), false);
                return points;
            }

            for (JsonNode node : nodes) {
                double x = node.path("x").asDouble();
                double y = node.path("y").asDouble();
                double heading = node.path("heading").asDouble();
                double vx = node.path("vx").asDouble();
                double vy = node.path("vy").asDouble();
                double omega = node.path("omega").asDouble();
                double ax = node.path("ax").asDouble();
                double ay = node.path("ay").asDouble();

                PathPoint point = new PathPoint(
                    new Pose2d(x, y, Rotation2d.fromRadians(heading)), 
                    new PointConstraints(vx, vy, ax, ay, RadiansPerSecond.of(omega))
                );

                points.add(point);
            }
        } catch (IOException e) {
            DriverStation.reportError("Failed to load Choreo trajectory into Pure-Pursuit!", e.getStackTrace());
        }

        System.out.println("Sucessfully loaded Choreo Trajectory, name: " + trajectoryName + ".");
        return points;
    }
    
}
