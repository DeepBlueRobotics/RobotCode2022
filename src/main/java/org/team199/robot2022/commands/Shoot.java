// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.ColorSensor;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shoot extends CommandBase {
  
  private final ColorSensor colorSensor;
  private final Shooter shooter;

  /** Creates a new Regurgitate. 
   * 
   * **/
  public Shoot(ColorSensor colorSensor, Shooter shooter) {
    //addRequirements(requirements);
    // Use addRequirements() here to declare subsystem dependencies.
    /**
     * takes in detectcolor method output from the sensor readings
     * 
     */
    addRequirements(this.colorSensor = colorSensor, this.shooter = shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /**
     * checks detectcolor input to see if the ball is correct color, 
     * if correct, the command is not run
     * otherwise if not, the command is run
     */
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /**
     * Here we want to essentially stop and then reverse the motors in order 
     * to regurgitate the balls from the robot
     */

    // from shooter team: 
    // Uses color sensor
    // Shooter team wants to make it so that it automatically checks what ball is in the shooter. 
    // If its the wrong ball, it can automatically turn and shoot the other direction without any input from the driver. 
    // possibly maybe just shoot the ball at a low speed
    // If its right, it can automatically shoot.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /**
     * will end when the ball is correct color again or the incorrect color
     * is no longer read
     * 
     * **IMPORTANT NOTE**
     * we may need a second color sensor or some other kind of sensor
     * to check if the ball has left the robot and been successfully 
     * regurgitated
     */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}