// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ExtendClimber extends ParallelCommandGroup {

    public ExtendClimber(Climber climber) {
        super(
            new FunctionalCommand(
                () -> {},
                climber::extendLeft,
                climber::stopLeft,
                climber::isLeftExtended
            ),
            new FunctionalCommand(
                () -> {},
                climber::extendRight,
                climber::stopRight,
                climber::isRightExtended
            )
        );
        addRequirements(climber);
    }

}
