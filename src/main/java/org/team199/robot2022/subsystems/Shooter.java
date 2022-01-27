// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot() {
    // Shoots the ball into the goal
    // Shoot method should already check if the ball is the correct color
    
  }

  public void miss() {
    // Shoots the ball away from the goal
    // Shoot command should already check if the ball is the wrong color
  }
}
