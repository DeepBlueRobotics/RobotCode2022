// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Shoots one ball (is called with while pressed AND when pressed)
package org.team199.robot2022.commands;

import static org.mockito.ArgumentMatchers.nullable;

import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;
import org.team199.robot2022.subsystems.Shooter.ShootMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Shoot extends SequentialCommandGroup {

  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;
  private final Drivetrain dt;
  private boolean detectedBall = false;
  private ShootMode shootMode = null;
  
  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter, Drivetrain dt) {
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter, this.dt = dt);
  }

  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter, Drivetrain dt, ShootMode mode)
  {
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter, this.dt = dt);
    shootMode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  //Shoots a ball forward/backwards depending on ball color
  public void execute() { // TODO : Determine when to shoot for lower goal
    if (shootMode != null)
    {
      shooter.setMainSpeed(shootMode);
      if (shooter.isAtTargetSpeed())
      {
        intakeFeeder.invertAndRun(Motor.TOP, false, true);
      }

    } else {
      if (intakeFeeder.getCargo().peekLast()){ //if color is correct
        // Shoot the ball
        shooter.setMainSpeed(ShootMode.UPPER);
        if (shooter.isAtTargetSpeed()) {
          intakeFeeder.invertAndRun(Motor.TOP, false, true);
        }
      } else {
        // turn 90 degrees and soft shoot
        shooter.setMainSpeed(ShootMode.SOFT);
        if (shooter.isAtTargetSpeed()) {
          intakeFeeder.invertAndRun(Motor.TOP, false, true);
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) return;
    intakeFeeder.invertAndRun(Motor.TOP, false, false);
    intakeFeeder.popSecondBall();
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
  
  /*
  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter) {
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
  */
  
}
