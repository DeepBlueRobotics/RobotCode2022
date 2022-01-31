package org.team199.robot2022;

import frc.robot.lib.path.RobotPath;

public class AutoPath {

    public final boolean shootAtStart;
    public final RobotPath path1;
    public final RobotPath path2;
    public final boolean shootAtEnd;
    public final boolean runIntake;

    public AutoPath(boolean shootAtStart, RobotPath path1, RobotPath path2, boolean shootAtEnd, boolean runIntake) {
        this.shootAtStart = shootAtStart;
        this.path1 = path1;
        this.path2 = path2;
        this.shootAtEnd = shootAtEnd;
        this.runIntake = runIntake;
    }

}
