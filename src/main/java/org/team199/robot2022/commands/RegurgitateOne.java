// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class RegurgitateOne extends CommandBase {
  private final IntakeFeeder intakeFeeder;

  public RegurgitateOne(IntakeFeeder intakeFeeder) {
    addRequirements(this.intakeFeeder = intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(intakeFeeder.getCargo().size())
    {
      case 1:
        intakeFeeder.invertAndRun(Motor.MIDDLE, true, true);
        intakeFeeder.invertAndRun(Motor.BOTTOM, true, true);
        break;          
      case 2:
        intakeFeeder.invertAndRun(Motor.BOTTOM, true, true);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (intakeFeeder.getCargo().size() > 0)
      intakeFeeder.popSecondBall();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch(intakeFeeder.getCargo().size())
    {
      case 1:
        if (intakeFeeder.isBallThere(Motor.MIDDLE) || intakeFeeder.isBallThere(Motor.BOTTOM))
          return false;
        else return true;
      case 2:
        if (intakeFeeder.isBallThere(Motor.BOTTOM))
          return false;
        else return true;
      default:
        return true;
    }
  }
}
