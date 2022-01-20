// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2022.subsystems.ColorSensor;

public class DetectColor extends CommandBase {

  private ColorSensor colorSensor;
  /** Creates a new DetectColor. */
  public DetectColor(ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.colorSensor = colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /**
     * may want to add WHAT COLOR BALLS THE ROBOT WANTS TO HAVE
     * change the boolean output for false to match the correct color
     */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSensor.detectColor();
    /**
     * once the color is read, it will output to the smartdashboard
     * we can then use this information and put it into regurgitate
     * 
     * - create a boolean that detects if a ball is "correct", this boolean 
     * can then be switched around depending on what team we are.
     *    - ideally, we want the ball to be instantly regurgitated if it is not
     *      the "correct" color
     *    
     * 
     * 
     */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /**
     * I think we want this command to end when the robot stops running
     * so we will have the sensor running from start of auto to end of teleop
     */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
