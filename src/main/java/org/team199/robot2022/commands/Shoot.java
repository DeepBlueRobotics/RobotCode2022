// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import com.revrobotics.CANSparkMax;

import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.MotorControllerFactory;

public class Shoot extends CommandBase {
  
  private final IntakeFeeder intakeFeeder;
  private final Shooter shooter;

  
  public Shoot(IntakeFeeder intakeFeeder, Shooter shooter) {
    /**
     * takes in detectcolor method output from the sensor readings
     * 
     */
    addRequirements(this.intakeFeeder = intakeFeeder, this.shooter = shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.


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
    /**
     * will end when the ball is correct color again or the incorrect color
     * is no longer read
     * 
     * **IMPORTANT NOTE**

     * goint to use timing to detect when the ball has left the robot
     */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
