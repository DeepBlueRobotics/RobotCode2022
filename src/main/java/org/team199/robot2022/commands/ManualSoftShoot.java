// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;
import org.team199.robot2022.subsystems.Shooter.ShootMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualSoftShoot extends CommandBase {
  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;
  private boolean detectedBall = false;
  
  public ManualSoftShoot(IntakeFeeder intakeFeeder, Shooter shooter) {
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setMainSpeed(ShootMode.SOFT);
    if (shooter.isAtTargetSpeed())
      intakeFeeder.invertAndRun(Motor.TOP, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeFeeder.invertAndRun(Motor.TOP, false, false);
    intakeFeeder.pollBall();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.isBallThere())
      detectedBall = true;
    if (detectedBall && !shooter.isBallThere()) {
      detectedBall = false;
      return true;
    }
    return false;
  }
}
