package org.team199.robot2022;

import frc.robot.lib.path.RobotPath;
import java.util.List;

public class AutoPath {

    public final boolean shootAtStart;
    public final List<RobotPath> path;
    public final boolean shootAtEnd;
    public final boolean runIntake;

    public AutoPath(boolean shootAtStart, List<RobotPath> path, boolean shootAtEnd, boolean runIntake) {
        this.shootAtStart = shootAtStart;
        this.path = path;
        this.shootAtEnd = shootAtEnd;
        this.runIntake = runIntake;
    }

}
