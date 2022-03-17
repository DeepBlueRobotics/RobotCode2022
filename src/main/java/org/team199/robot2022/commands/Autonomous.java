package org.team199.robot2022.commands;

import org.team199.robot2022.AutoPath;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.path.RobotPath;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(AutoPath path, boolean shootAtStart, boolean shootAtEnd, Drivetrain drivetrain, Shooter shooter) {
        addRequirements(drivetrain, shooter);

        addCommands(
            new InstantCommand(path.path.get(0)::initializeDrivetrainPosition),
            shootAtStart ? /* new Shoot1(intakeFeeder, shooter) */ new InstantCommand() : new InstantCommand()
        );
        for (int i = 0; i < path.path.size(); i ++){
            addCommands(path.path.get(i).getPathCommand(false, i == path.path.size()-1));
        }
        addCommands(
            shootAtEnd ? new SequentialCommandGroup(/* new Shoot1(intakeFeeder, shooter), new Shoot1(intakeFeeder, shooter) */) : new InstantCommand()
        );
    }

}
