// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RetractClimber extends ParallelCommandGroup {
    private final Climber climber;

    public RetractClimber(Climber climber) {
        addRequirements(this.climber = climber);
    }

    @Override
    public void initialize() {
        climber.moveMotors(climber.MotorSpeed.kRetractSpeed,climber.bothMotors);
    }

    @Override
    public boolean isFinished(){
        return isMotorRetracted(climber.bothMotors);
    }

    @Override
    public void end(){
        climber.stopMotors(climber.bothMotors);
    }

}
