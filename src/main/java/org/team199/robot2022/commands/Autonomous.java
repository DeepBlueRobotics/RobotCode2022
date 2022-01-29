package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.ColorSensor;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.path.RobotPath;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(boolean shootAtStart, RobotPath path1, RobotPath path2, boolean shootAtEnd, boolean runIntake, Drivetrain drivetrain, /*IntakeFeeder intakeFeeder,*/ ColorSensor colorSensor, Shooter shooter) {
        addRequirements(drivetrain, /*intakeFeeder,*/ colorSensor, shooter);

        addCommands(
            new InstantCommand(path1::initializeDrivetrainPosition),
            shootAtStart ? /* new Shoot1(intakeFeeder, shooter) */ new InstantCommand() : new InstantCommand(),
            runIntake ? /* new DeployIntake(intakeFeeder) */ new InstantCommand() : new InstantCommand(),
            path1.getPathCommand(true, path2 == null),
            runIntake ? /* new RetractIntake(intakeFeeder) */ new InstantCommand() : new InstantCommand(),
            path2 == null ? new InstantCommand() : new SequentialCommandGroup(new InstantCommand(path2::initializeDrivetrainPosition), path2.getPathCommand(true, true)),
            shootAtEnd ? new SequentialCommandGroup(/* new Shoot1(intakeFeeder, shooter), new Shoot1(intakeFeeder, shooter) */) : new InstantCommand()
        );
    }

}
