// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;
import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MoveToClimb extends CommandBase {

  private final Drivetrain drivetrain;
  //Get and save initial position

  /** Creates a new SetMidRungClimber. */
  public MoveToClimb(Drivetrain drivetrain) {
    addRequirements(this.drivetrain = drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(Constants.DriveConstants.driveToRungSpeed, 0, 0);
    //For full extension rotate 135 full rotations of motor (70 in of rope)
    //Should retract same amount
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //If new position is far enough from init position, then return true, else return false
    return false;
  }
}
