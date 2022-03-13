// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RetractClimber extends ParallelCommandGroup {

    public RetractClimber(Climber climber, boolean isSlow) {
        super(
            new FunctionalCommand(
                () -> {},
                climber::retractLeft,
                climber::stopLeft,
                climber::isLeftRetracted
            ),
            new FunctionalCommand(
                () -> {},
                climber::retractRight,
                climber::stopRight,
                climber::isRightRetracted
            )
        );
        addRequirements(climber);
    }

}
