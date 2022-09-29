// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class RetractClimber extends ParallelCommandGroup {

    public RetractClimber(Climber climber) {
        super(
            new FunctionalCommand(
              () -> {},
              () -> climber.moveMotors(Climber.MotorSpeed.retract,Climber.rightMotor),
              (interrupted) -> climber.stopMotors(Climber.rightMotor),
              () -> climber.isMotorRetracted(Climber.rightMotor)
            ),
            new FunctionalCommand(
              () -> {},
              () -> climber.moveMotors(Climber.MotorSpeed.retract,Climber.leftMotor),
              (interrupted) -> climber.stopMotors(Climber.leftMotor),
              () -> climber.isMotorRetracted(Climber.leftMotor)
            )
        );
        addRequirements(climber);
    }
}
