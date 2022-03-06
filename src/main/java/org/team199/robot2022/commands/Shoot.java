// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Shoots one ball (is called with while pressed AND when pressed)
package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Shoot extends SequentialCommandGroup {

  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;

  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter) {
    /**
     * takes in detectcolor method output from the sensor readings
     */
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter);

    addCommands(
      new WaitUntilCommand(shooter::isAtTargetSpeed),
      new FunctionalCommand(
        () -> {},
        intakeFeeder::runForward,
        interrupted -> {
          if(interrupted) return;
          intakeFeeder.stopRunningFeeder();
          intakeFeeder.popFirstBall();
        },
        () -> !shooter.isAtTargetSpeed(),
        intakeFeeder
      )
    );
  }
}
