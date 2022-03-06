// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Shoots one ball (is called with while pressed AND when pressed)
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
    //set initial # of balls in feeder
  }

  // Called every time the scheduler runs while the command is scheduled.


  @Override
  //Shoots a ball forward/backwards depending on ball color
  public void execute() {
    
    if (intakeFeeder.eject())
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
    // after finished shooting once return true 
      // if the initial #balls in feeder is == to current #balls in feeder, return false
    
    // not a serious comment
    // 20 minutes of arguing, and all we learned is that we need programmer socks
    return false;
  }
}
