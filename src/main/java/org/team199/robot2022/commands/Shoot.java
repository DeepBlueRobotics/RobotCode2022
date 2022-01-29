// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;
  
  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter) {
    /**
     * takes in detectcolor method output from the sensor readings
     */
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  /** Automatically checks what ball is in the shooter. 
    * If its the wrong ball, it can automatically turn and shoot the ball at a low speed to miss
    * If its right, it can automatically shoot.
    */
  @Override
  public void execute() {
    /**
     * checks detectcolor input to see if the ball is correct color, 
     * if correct, the command is not run
     * otherwise if not, the command is run
     */
    
    if (intakeFeeder.eject())
    {
      // Shoot the ball, the color is correct or color sensor not work
    }
    else
    {
      // Regurgitate the ball, the color is incorrect
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // The command should end when there are no more balls in feeder/shooter
    return false;
  }
}
