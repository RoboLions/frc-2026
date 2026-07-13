package frc.robot.lib.auto;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;
import frc.robot.lib.auto.Pursuiter.util.Stopwatch;

public class AutoPathBase {

    private PursuitProfile profile;
    private PursuitPath startPath;
    private Stopwatch stopwatch = new Stopwatch();
    private Command autoCommand;
    private String pathName;

    public AutoPathBase (PursuitProfile profile, String pathName) {
        this.profile = profile;
        this.pathName = pathName;
    }

    public void setStartPath(PursuitPath startPath) {
        this.startPath = startPath;
    }

    public PursuitProfile getProfile() {
        return this.profile;
    }

    public Command asCommand() {
        return this.autoCommand;
    }

    public void logPathTrajectory(int density) {
        Logger.recordOutput("Pursuiter/" + pathName + " Trajectory", startPath.getVisualizedPath(density));
    }

    public void logCurrentPathPoints() {
        Logger.recordOutput("Pursuiter/" + pathName + " LookAhead", startPath.getLookAhead().point());
        Logger.recordOutput("Pursuiter/" + pathName + " CurrentPoint", startPath.getCurrentPoint().point());
    }

}
