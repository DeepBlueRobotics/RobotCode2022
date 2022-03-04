// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Shoots one ball (is called with while pressed AND when pressed)
package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;
import org.team199.robot2022.subsystems.Shooter.ShootMode;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;
  private final Drivetrain dt;
  private boolean detectedBall = false;
  
  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter, Drivetrain dt) {
    /**
     * takes in detectcolor method output from the sensor readings
     * 
     */
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter, this.dt = dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  @Override
  //Shoots a ball forward/backwards depending on ball color
  public void execute() {
    if (intakeFeeder.eject()){ //if color is correct
      // Shoot the ball
      shooter.setMainSpeed(ShootMode.UPPER); // TODO : Determine when to shoot for lower goal
      if (shooter.isAtTargetSpeed()) {
        // TODO : Make sure that by the time the ball makes contact with shooter motors,
        // that is is out of range of top motor
        intakeFeeder.invertAndRun(Motor.TOP, false, true);
      }
    } else {
      //turn around and soft shoot
      shooter.setMainSpeed(ShootMode.SOFT);
      if (shooter.isAtTargetSpeed()) {
        intakeFeeder.invertAndRun(Motor.TOP, false, true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeFeeder.invertAndRun(Motor.TOP, false, false);
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
