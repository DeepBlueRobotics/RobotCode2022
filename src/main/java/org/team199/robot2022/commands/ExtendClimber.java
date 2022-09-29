// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class ExtendClimber extends ParallelCommandGroup {

    public ExtendClimber(Climber climber) {
        super(
            new FunctionalCommand(
                () -> {},
                () -> climber.moveMotors(Climber.MotorSpeed.extend,Climber.rightMotor),
                (interrupted) -> climber.stopMotors(Climber.rightMotor),
                () -> climber.isMotorExtended(Climber.rightMotor)
            ),
            new FunctionalCommand(
                () -> {},
                () -> climber.moveMotors(Climber.MotorSpeed.extend,Climber.leftMotor),
                (interrupted) -> climber.stopMotors(Climber.leftMotor),
                () -> climber.isMotorExtended(Climber.leftMotor)
            )
        );
        addRequirements(climber);
    }
}
