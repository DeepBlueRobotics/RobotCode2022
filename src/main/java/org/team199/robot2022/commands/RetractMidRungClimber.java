// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.team199.robot2022.subsystems.Climber;

public class RetractMidRungClimber extends CommandBase {

  private final Climber climber;  

  /** Creates a new RetractMidRungClimber. */
  public RetractMidRungClimber(Climber climber) {
    addRequirements(this.climber = climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.climb(Climber.kRetractSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (climber.getRotations() <= Climber.kMidRungRetractions) {
      return true;
    } else {
      return false;
    }
  }
}
