// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;

public class IntakeFeeder extends SubsystemBase {

  private final CANSparkMax feederBottom = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
  private final CANSparkMax feederTop = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);

  private final double speed = 1.0;
  
  /** Creates a new IntakeFeeder. */
  public IntakeFeeder() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    //moves intake motors
  }

  public void feeder() {
    feederBottom.set(speed);
    feederTop.set(speed);
  }
}
