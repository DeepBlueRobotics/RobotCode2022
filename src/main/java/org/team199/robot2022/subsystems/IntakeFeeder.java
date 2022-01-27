// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;

import org.team199.robot2022.Constants;

//import org.team199.robot2021.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
//import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.logging.Log;

public class IntakeFeeder extends SubsystemBase {
  /** Creates a new IntakeFeeder. */
  public IntakeFeeder() {
    
  }

  public double motorSpeed; //set to some arbitrary value for now
  public double timeRunning;
  public int stageNumber;


  private final createSparkMax topMotor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeTop);
  private final createSparkMax middleMotor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeMiddle);
  private final createSparkMax bottomMotor = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeBottom);
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intake() {
    //moves intake motors
    runBottomMotor(motorSpeed);
    runMiddleMotor(motorSpeed);
    runTopMotor(motorSpeed);
  }

  

  public void runBottomMotor(double motorSpeed)
  {
    //runs bottommost intake motor
  }

  public void runMiddleMotor(double motorSpeed)
  {
    //runs middle intake motor
  }

  public void runTopMotor(double motorSpeed)
  {
    //runs top motor
  }

  public void setSpeed(double newSpeed)
  {
    motorSpeed = newSpeed;
  }
}
