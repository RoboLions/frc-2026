package frc.robot.lib.auto.paths;

import frc.robot.lib.auto.AutoPathBase;
import frc.robot.lib.auto.Pursuiter.PursuitPath;
import frc.robot.lib.auto.Pursuiter.PursuitProfile;

public class LeftSideTwoTrip extends AutoPathBase {

    private PursuitPath L1 = new PursuitPath(getProfile(), "L1.traj");
    private PursuitPath L2 = new PursuitPath(getProfile(), "L2.traj");
    private PursuitPath L3 = new PursuitPath(getProfile(), "L3.traj");

    public LeftSideTwoTrip(PursuitProfile profile) {
        super(profile, "LeftSideTwoTrip");

        L1.append(L2);
        L1.append(L3);
    }
    
}
