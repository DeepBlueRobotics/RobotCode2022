package org.team199.robot2022;

import org.carlmontrobotics.lib199.path.RobotPath;
import java.util.List;

import org.team199.robot2022.subsystems.Shooter.ShotPosition;

public class AutoPath {

    public final boolean shootAtStart;
    public final List<RobotPath> path;
    public final boolean shootAtEnd;
    public final boolean runIntake;
    public final boolean useLimelight;
    public final ShotPosition startShotPosition, endShotPosition;

    public AutoPath(boolean shootAtStart, List<RobotPath> path, boolean shootAtEnd, boolean runIntake, boolean useLimelight) {
        this(shootAtStart, path, shootAtEnd, runIntake, useLimelight, ShotPosition.FENDER, ShotPosition.FENDER);
    }

    public AutoPath(boolean shootAtStart, List<RobotPath> path, boolean shootAtEnd, boolean runIntake, boolean useLimelight, ShotPosition startShotPosition, ShotPosition endShotPosition) {
        this.shootAtStart = shootAtStart;
        this.path = path;
        this.shootAtEnd = shootAtEnd;
        this.runIntake = runIntake;
        this.useLimelight = useLimelight;
        this.startShotPosition = startShotPosition;
        this.endShotPosition = endShotPosition;
    }

}
