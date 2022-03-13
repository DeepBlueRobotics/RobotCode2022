package org.team199.robot2022.commands;

import org.team199.robot2022.AutoPath;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.path.RobotPath;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(AutoPath path, Drivetrain drivetrain, IntakeFeeder intakeFeeder, Shooter shooter) {
        this(path.shootAtStart, path.path1, path.path2, path.shootAtEnd, path.runIntake, drivetrain, intakeFeeder, shooter);
    }

    public Autonomous(boolean shootAtStart, RobotPath path1, RobotPath path2, boolean shootAtEnd, boolean runIntake, Drivetrain drivetrain, IntakeFeeder intakeFeeder, Shooter shooter) {
        addRequirements(drivetrain, intakeFeeder, shooter);

        addCommands(
            new InstantCommand(path1::initializeDrivetrainPosition),
            shootAtStart ? /* new Shoot1(intakeFeeder, shooter) */ new InstantCommand() : new InstantCommand(),
            runIntake ? /* new DeployIntake(intakeFeeder) */ new InstantCommand() : new InstantCommand(),
            path1.getPathCommand(false, path2 == null),
            runIntake ? /* new RetractIntake(intakeFeeder) */ new InstantCommand() : new InstantCommand(),
            path2 == null ? new InstantCommand() : new SequentialCommandGroup(new InstantCommand(path2::initializeDrivetrainPosition), path2.getPathCommand(false, true)),
            shootAtEnd ? new SequentialCommandGroup(/* new Shoot1(intakeFeeder, shooter), new Shoot1(intakeFeeder, shooter) */) : new InstantCommand()
        );
    }

}
