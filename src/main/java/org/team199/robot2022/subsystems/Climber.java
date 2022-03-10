// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static double kMidRungRotations = 133.7;
  public static double kMidRungRetractions = 0;
  //TODO : Set this
  public static double kRetractSpeed = 0.4;
  //TODO : Set this
  public static double kExtendSpeed = 0.05;

  //TODO : set these to correct forwards backwards values idk how the motors work
  public final boolean leftInverted = true;
  public final boolean rightInverted = false;

  // TODO : SET THESE TO CORRECT VALUES
  public final double extendPosition = 100;
  public final double retractPosition = extendPosition/2;
  public final double bottomPosition = 0;

  private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
  private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);

  public Climber() {
    
    
    /**
     * not really sure what these do but they're probably important as
     * climber is gonna have a lot of the same things as shooter
     * 
    left.setSmartCurrentLimit(40);
    right.setSmartCurrentLimit(40);
    left.follow(right, true);
    left.setInverted(true);
     */

    //"for now I am just using setInverted for both" -> changed to only right inversion
    //left.setInverted(leftInverted);
    right.setInverted(rightInverted);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climb(double speed) {
    //TODO : Rotate motor
    
    left.set(kExtendSpeed);

  }

  public void runLeft(double speed)
  {
    //TODO : Rotate motor
    left.set(speed);
  }

  public void runRight(double speed)
  {
    //TODO : Rotate motor
    right.set(speed);
  }

  public void runBackwards()
  {
    left.setInverted(!leftInverted);
    right.setInverted(!rightInverted);
    left.set(kRetractSpeed);
    right.set(kRetractSpeed);
    
  }

  public double getRotations() {
    //TODO : Return how much the climber motor has rotated using encoders
    //Should return a value where 1 = 1 whole rotation

    return 0;
  }
  

  //GETTERS
  public double getMotorPosition(CANSparkMax motor)
  {
    return motor.getEncoder().getPosition();
  }

  public CANSparkMax getLeft()
  {
    return left;
  }

  public CANSparkMax getRight()
  {
    return right;
  }

  public boolean checkLeftExtend()
  {
    return left.getEncoder().getPosition() >= extendPosition;
  }

  public boolean checkRightExtend()
  {
    return right.getEncoder().getPosition() >= extendPosition;
  }

  public boolean checkLeftRetract()
  {
    return left.getEncoder().getPosition() <= retractPosition;
  }

  public boolean checkRightRetract()
  {
    return right.getEncoder().getPosition() <= retractPosition;
  }

  public double getExtendSpeed()
  {
    return kExtendSpeed;
  }

  public double getRetractSpeed()
  {
    return kRetractSpeed;
  }

  
}
