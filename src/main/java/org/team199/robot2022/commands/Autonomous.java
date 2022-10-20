package org.team199.robot2022.commands;

import java.sql.Time;

import org.team199.robot2022.AutoPath;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import org.carlmontrobotics.lib199.Limelight;
import org.carlmontrobotics.lib199.path.RobotPath;

public class Autonomous extends SequentialCommandGroup {

    public Autonomous(AutoPath path, boolean shootAtStart, boolean shootAtEnd, Drivetrain drivetrain, Shooter shooter, IntakeFeeder intakeFeeder, Limelight lime) {
        addRequirements(drivetrain, shooter, intakeFeeder);
        addCommands(new PassiveManualIntake(intakeFeeder));
        if(path.shootAtStart) addCommands(new InstantCommand(() -> shooter.setShotPosition(path.startShotPosition)));

        addCommands(
            new InstantCommand(path.path.get(0)::initializeDrivetrainPosition),
            shootAtStart ? new SequentialCommandGroup( new WaitUntilCommand(shooter::isAtTargetSpeed),  new WaitCommand(1.5), new Shoot(intakeFeeder, shooter)) : new InstantCommand(),
            shootAtStart ? new SequentialCommandGroup( new WaitUntilCommand(shooter::isAtTargetSpeed),  new WaitCommand(1.5), new Shoot(intakeFeeder, shooter)) : new InstantCommand()
        );
        //addCommands(new WaitCommand(4));
        for (int i = 0; i < path.path.size(); i++){
            addCommands(new ParallelRaceGroup(path.path.get(i).getPathCommand(false, false), new PassiveManualIntake(intakeFeeder).perpetually()));
            if (i < path.path.size()-1){
                if(path.useLimelight && false)
                    addCommands(
                        new ParallelRaceGroup(
                            new TeleopDrive(drivetrain, () -> 0D, () -> 0D, () -> 0D, () -> false, () -> true, lime), // Auto-pickup is sketch
                            new WaitUntilCommand(() -> intakeFeeder.getNumBalls() == 1), // Wait until we pick something up
                            new SequentialCommandGroup(
                                new WaitCommand(5), // If we haven't found anything after 5 seconds, abort
                                new ConditionalCommand(
                                    new WaitUntilCommand(() -> intakeFeeder.getNumBalls() == 1), // We're chasing a ball! Wait to pick it up
                                    new InstantCommand(), // We can't see the ball! Just abort
                                    () -> NetworkTableInstance.getDefault().getTable(lime.config.ntName).getEntry("tv").getDouble(0) == 1
                                )
                            )
                        )
                    );
                else
                    addCommands(new InstantCommand(path.path.get(i+1)::initializeDrivetrainPosition));
            }
        }
        addCommands(new InstantCommand(() -> {drivetrain.stop();}));


        // if(path.shootAtEnd) addCommands(new InstantCommand(() -> shooter.setShotPosition(path.endShotPosition)));
        // addCommands(new ParallelCommandGroup(new TeleopDrive(drivetrain, () -> 0D, () -> 0D, () -> 0D, () -> false), new RunCommand(intakeFeeder::runForward, intakeFeeder)));
        addCommands(new ParallelCommandGroup(new RunCommand(drivetrain::stop, drivetrain), new RunCommand(intakeFeeder::runForward, intakeFeeder)));

        addCommands(
            shootAtEnd ? new SequentialCommandGroup( new WaitUntilCommand(shooter::isAtTargetSpeed),  new WaitCommand(0.4), new Shoot(intakeFeeder, shooter), new Shoot(intakeFeeder, shooter)) : new InstantCommand()
        );
    }

}
