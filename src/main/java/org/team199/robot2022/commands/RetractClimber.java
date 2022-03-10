// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;
import com.revrobotics.CANSparkMax;

import org.team199.robot2022.subsystems.Climber;


import edu.wpi.first.wpilibj2.command.CommandBase;


public class RetractClimber extends CommandBase {
  /** Creates a new ExtendClimber. */
  private final Climber climber;
  private final double extendSpeed;
  private final double retractSpeed;

  public RetractClimber(Climber climber) {
    addRequirements(this.climber = climber);
    extendSpeed = climber.getExtendSpeed();
    retractSpeed = -climber.getRetractSpeed();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Runs motors until they hit the correct height
    
    if (!climber.checkLeftRetract()) {
      climber.runLeft(retractSpeed);
    }
    if (!climber.checkRightRetract()) {
      climber.runRight(retractSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.checkLeftRetract() && climber.checkRightRetract())
    {
      return true;
    }
    else
    { 
      return false;
    }
  }
}
