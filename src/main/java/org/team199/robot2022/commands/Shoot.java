// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Shoots one ball (is called with while pressed AND when pressed)
package org.team199.robot2022.commands;

import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.ColorSensor;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  private final ColorSensor colorSensor;
  private final Shooter shooter;
  
  public Shoot(ColorSensor colorSensor, Shooter shooter) {
    /**
     * takes in detectcolor method output from the sensor readings
     */
    addRequirements(this.colorSensor = colorSensor, this.shooter = shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set initial # of balls in feeder
  }

  /** Automatically checks what ball is in the shooter. 
    * If its the wrong ball, it can automatically turn and shoot the ball at a low speed to miss
    * If its right, it can automatically shoot.
    */
  @Override
  //Shoots a ball forward/backwards depending on ball color
  public void execute() {
    
    if (colorSensor.detectColor()) //if color is correct
    {
      // Shoot the ball
      if (shooter.isAtTargetSpeed()) {
        // run feeder
      }
    }
    else
    {

    }
    //update current # of balls
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // after finished shooting once return true 
      // if the initial #balls in feeder is == to current #balls in feeder, return false
    
    // not a serious comment
    // 20 minutes of arguing, and all we learned is that we need programmer socks
    return false;
  }
}
