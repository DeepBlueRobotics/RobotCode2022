// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import org.team199.robot2022.Constants;


public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static double kMidRungRotations = 133.7;
  public static double kMidRungRetractions = 0;
  //TODO : Set this
  public static double kRetractSpeed = 0;
  //TODO : Set this
  public static double kExtendSpeed = 0;

  //TODO : set these to correct forwards backwards values idk how the motors work
  public final boolean leftInverted = true;
  public final boolean rightInverted = false;

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

    //for now I am just using setInverted for both
    left.setInverted(leftInverted);
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

  public void runForwards()
  {
    //TODO : Rotate motor
    
    left.set(kExtendSpeed);
    right.set(kExtendSpeed);


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
  
}
