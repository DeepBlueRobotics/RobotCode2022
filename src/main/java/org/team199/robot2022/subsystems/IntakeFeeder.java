// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeFeeder extends SubsystemBase {
  /** Creates a new IntakeFeeder. */
  public IntakeFeeder() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    //moves intake motors
  }

  public void runFirstMotor()
  {
    //runs bottommost intake motor
  }

  public void runSecondMotor()
  {
    //runs middle intake motor
  }

  public void runThirdMotor()
  {
    //runs top motor
  }
}
