// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static double kMidRungRotations = 133.7;
  public static double kMidRungRetractions = 0;
  //TODO : Set this
  public static double kRetractSpeed = 0;
  //TODO : Set this
  public static double kExtendSpeed = 0;

  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    //TODO : Rotate motor

  }

  public double getRotations() {
    //TODO : Return how much the climber motor has rotated using encoders
    //Should return a value where 1 = 1 whole rotation
    return 0;
  }
  
}
