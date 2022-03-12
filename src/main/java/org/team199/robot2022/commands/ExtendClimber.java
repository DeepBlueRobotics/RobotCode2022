// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public class ExtendClimber extends FunctionalCommand {

    public ExtendClimber(Climber climber) {
        super(
            () -> {},
            climber::extend,
            climber::stop,
            climber::isExtended,
            climber
        );
    }

}
