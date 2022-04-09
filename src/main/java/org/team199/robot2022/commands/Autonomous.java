package org.team199.robot2022.commands;

import java.sql.Time;

import org.team199.robot2022.AutoPath;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.lib.path.RobotPath;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(AutoPath path, boolean shootAtStart, boolean shootAtEnd, Drivetrain drivetrain, Shooter shooter, IntakeFeeder intakeFeeder) {
        addRequirements(drivetrain, shooter, intakeFeeder);
        addCommands(new PassiveAutomaticIntake(intakeFeeder));
        if(path.shootAtStart) addCommands(new InstantCommand(() -> shooter.setShotPosition(path.startShotPosition)));

        addCommands(
            new InstantCommand(path.path.get(0)::initializeDrivetrainPosition),
            shootAtStart ? new SequentialCommandGroup( new WaitUntilCommand(shooter::isAtTargetSpeed),  new WaitCommand(1.5), new Shoot(intakeFeeder, shooter)) : new InstantCommand()
        );
        //addCommands(new WaitCommand(4));
        for (int i = 0; i < path.path.size(); i++){
            addCommands(new ParallelRaceGroup(path.path.get(i).getPathCommand(false, false), new PassiveAutomaticIntake(intakeFeeder).perpetually()));
            if (i < path.path.size()-1){
            addCommands(new InstantCommand(path.path.get(i+1)::initializeDrivetrainPosition));
            }
        }
        addCommands(new InstantCommand(() -> {drivetrain.stop();}));


        // if(path.shootAtEnd) addCommands(new InstantCommand(() -> shooter.setShotPosition(path.endShotPosition)));
        addCommands(new RunCommand(intakeFeeder::runForward, intakeFeeder));

        addCommands(
            shootAtEnd ? new SequentialCommandGroup( new WaitUntilCommand(shooter::isAtTargetSpeed),  new WaitCommand(0.4), new Shoot(intakeFeeder, shooter), new Shoot(intakeFeeder, shooter)) : new InstantCommand()
        );
    }

}
