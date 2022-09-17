// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ExtendClimber extends CommandBase {
    private final Climber climber;

    public ExtendClimber(Climber climber) {
        addRequirements(this.climber = climber);
    }

    @Override
    public void initialize() {
        climber.moveMotors(climber.MotorSpeed.kExtendSpeed,climber.Motor.both);
    }

    @Override
    public boolean isFinished(){
        return isMotorExtended(climber.Motor.both);
    }

    @Override
    public void end(){
        climber.stopMotors(climber.Motor.both);
    }

}
