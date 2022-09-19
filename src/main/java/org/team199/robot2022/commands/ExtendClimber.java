// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ExtendClimber extends CommandBase {
    private Climber climber;

    public ExtendClimber(Climber climber) {
        addRequirements(this.climber = climber);
    }

    public void initialize() {
        climber.moveMotors(Climber.MotorSpeed.extend,climber.bothMotors);
    }

    public boolean isFinished(){
        return climber.isMotorExtended(climber.bothMotors);
    }

    public void end(){
        climber.stopMotors(climber.bothMotors);
    }

}
