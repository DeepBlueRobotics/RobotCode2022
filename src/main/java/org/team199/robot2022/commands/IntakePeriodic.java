// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.IntakeFeeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakePeriodic extends CommandBase {

  private final IntakeFeeder intakeFeeder;
  private final boolean inverted = true;
  private int feed;
  /** Creates a new ManualPeriodic. */
  public IntakePeriodic(IntakeFeeder intakeFeeder) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intakeFeeder = intakeFeeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  /*
  public void autonomousPeriodic()
  {
    feed = intakeFeeder.getCargo().size();
    if (intakeFeeder.getCargo().size() < 2)
    {
      intakeFeeder.runBotMotor();
    }

    if (intakeFeeder.getCargo().size() > 0 && intakeFeeder.getCargo().peekFirst() == false)
    {
      intakeFeeder.stopMidMotor();
      // Regurgitate via intake
      if (!intakeFeeder.isJammed(intakeFeeder.getBottom()) && intakeFeeder.isBallThere(intakeFeeder.getBottom())) {
        intakeFeeder.getBottom().setInverted(inverted);
        intakeFeeder.runBotMotor();
      } else {
        // TODO : cargo.removeFirst() may remove the red ball but the color sensor will not detect that the ball is not
        // regurgitated and will not add the ball
        intakeFeeder.getCargo().removeFirst();
        intakeFeeder.getBottom().setInverted(!inverted);
        intakeFeeder.runBotMotor();
      }
    }
    if (intakeFeeder.detectColor()) {
      if (intakeFeeder.getCargo().size() == 1 && intakeFeeder.getCargo().peekFirst()) {
        // while ball is still in color sensor range move the ball out to prevent jam
        // TODO : The ball might not reach the destination fast enough if second ball gets in
        if (!intakeFeeder.isBallThere(intakeFeeder.getMiddle()) && intakeFeeder.isBallThere(intakeFeeder.getTop()))
        {
            intakeFeeder.stopMidMotor();
            intakeFeeder.stopTopMotor();
        } 
        else {
          if (!intakeFeeder.isJammed(intakeFeeder.getMiddle()) && intakeFeeder.isBallThere(intakeFeeder.getMiddle()))
          {
            intakeFeeder.runMidMotor();
            intakeFeeder.runTopMotor();
          }
          else if(intakeFeeder.isBallThere(intakeFeeder.getMiddle()))
          {
            intakeFeeder.unJam(intakeFeeder.getMiddle(), intakeFeeder.getMidSpeed());
          }
        }
      }
      // If this ball is the second ball in the feeder
      else {
        // This is to prevent any more balls getting in
        intakeFeeder.stopTopMotor();
        intakeFeeder.stopMidMotor();
        intakeFeeder.stopBotMotor();
        intakeFeeder.getBottom().setInverted(!inverted);
      } 
    }
  }

  public void manualPeriodic()
  {
    int feed = intakeFeeder.getFeed();
    switch(feed)
    {
      case 0:
        intakeFeeder.getBottom().setInverted(!inverted);
        intakeFeeder.stopMidMotor();
        if (!intakeFeeder.motorJammed(intakeFeeder.getBottom()))
        {
          intakeFeeder.runBotMotor();
        }
        else
        {
          intakeFeeder.unJam(intakeFeeder.getBottom(), intakeFeeder.getBotSpeed());
        }
        
        break;
      case 1:
        if (!intakeFeeder.isJammed(intakeFeeder.getBottom()))
        {
          intakeFeeder.runBotMotor();
        }
        else
        {
          intakeFeeder.unJam(intakeFeeder.getBottom(), intakeFeeder.getBotSpeed());
        }
        
        if (!intakeFeeder.isBallThere(intakeFeeder.getMiddle()) && intakeFeeder.isBallThere(intakeFeeder.getTop()))
        {
          intakeFeeder.stopMidMotor();
          intakeFeeder.stopTopMotor();
        } else {
          if (!intakeFeeder.isJammed(intakeFeeder.getMiddle())) {
            intakeFeeder.uninvertBotMotor();
            intakeFeeder.runMidMotor();
            intakeFeeder.runTopMotor();
          }
          else if(intakeFeeder.isJammed(intakeFeeder.getMiddle()) && intakeFeeder.isBallThere(intakeFeeder.getMiddle()))
          {
            intakeFeeder.unJam(intakeFeeder.getMiddle(), intakeFeeder.getMidSpeed());
          }
        }
        break;
      case 2:
        intakeFeeder.stopBotMotor();
        intakeFeeder.stopMidMotor();
        intakeFeeder.stopTopMotor();
        intakeFeeder.uninvertBotMotor();
        break;
    }
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
