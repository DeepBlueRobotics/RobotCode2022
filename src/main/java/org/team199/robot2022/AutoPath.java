package org.team199.robot2022;

import frc.robot.lib.path.RobotPath;
import java.util.List;

import org.team199.robot2022.subsystems.Shooter.ShotPosition;

public class AutoPath {

    public final boolean shootAtStart;
    public final List<RobotPath> path;
    public final boolean shootAtEnd;
    public final boolean runIntake;
    public final ShotPosition startShotPosition, endShotPosition;

    public AutoPath(boolean shootAtStart, List<RobotPath> path, boolean shootAtEnd, boolean runIntake) {
        this(shootAtStart, path, shootAtEnd, runIntake, ShotPosition.FENDER, ShotPosition.FENDER);
    }

    public AutoPath(boolean shootAtStart, List<RobotPath> path, boolean shootAtEnd, boolean runIntake, ShotPosition startShotPosition, ShotPosition endShotPosition) {
        this.shootAtStart = shootAtStart;
        this.path = path;
        this.shootAtEnd = shootAtEnd;
        this.runIntake = runIntake;
        this.startShotPosition = startShotPosition;
        this.endShotPosition = endShotPosition;
    }

}
