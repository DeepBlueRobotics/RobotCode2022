package org.team199.robot2022.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;

import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import org.team199.robot2022.Constants;

import com.revrobotics.ColorMatchResult;

import java.util.LinkedList;
import java.util.Queue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;

public class IntakeFeeder extends SubsystemBase {

  /**
   * Adds a box to SmartDashboard to put in team color
   * takes in what color the team is from smart dashboard to be checked
   * with color sensor
   */
  boolean ignored = SmartDashboard.putString("Team Color", "");
  char teamColor = SmartDashboard.getString("Team Color", "").toUpperCase().toCharArray()[0];

  /** The color sensor can detect how far away the object is from the sensor
   *  This can be used to determine whether there is a ball or not
   *  This ranges from 0 to 2047 where the value is larger when an object is closer
   *  9.75 in
   */
  private final int minProxmity = 2000;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final CANSparkMax intake = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
  // These three motors are for the 
  private final CANSparkMax bottom = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
  private final CANSparkMax middle = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
  private final CANSparkMax top = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);

  private final double speed = 1.0;

  /* Concern:
   * Make sure that when the ball is going thru the feeder that there is enough space between the balls
   * DO NOT let them rub against each other otherwise a jam can occur
   */

  // Will store what color balls are in the feeder
  // true = team color, false = not team color
  private Queue<Boolean> cargo = new LinkedList<>();

  // Should the robot intake balls or not (should only be used when color sensor is not working)
  // true = intake, false = regurgitate
  // THIS IS ONLY FOR INTAKE NOT THE SHOOTER
  private boolean feed = true;

  public IntakeFeeder() {
    // If the color sensor does not respond
    if (!m_colorSensor.isConnected())
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);
    // Initially have these motors run
    intake.setInverted(false);
    intake.set(speed);
    bottom.set(speed);
  }
  
  @Override
  public void periodic() {
    // Ocassionally update the team color if the team put the wrong one by accident
    teamColor = SmartDashboard.getString("Team Color", "").toUpperCase().toCharArray()[0];

    // Imperative to inform the driver whether color sensor is working
    if (m_colorSensor.isConnected()) {
      if (cargo.size() < 2)
      {
        intake.setInverted(false);
        bottom.setInverted(false);
      }
      if (detectColor()) {
        // If this ball is the first ball in the feeder
        if (cargo.size() == 1) {
          // while ball is still in color sensor range move the ball out to prevent jam
          while (m_colorSensor.getProximity() >= minProxmity) {
            middle.set(speed);
            top.set(speed);
          }
          middle.set(0);
          top.set(0);
        }
        // If this ball is the second ball in the feeder
        else {
          // This is to prevent any more balls getting in
          intake.setInverted(true);
          bottom.setInverted(true);
        } 
      }
    }
    else {
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
      // Reset the balls in the cargo as color sensor no longer works and we cannot accurately record the cargo
      cargo = new LinkedList<>();
      // Manually intake balls
      if (feed)
      {
        intake.setInverted(false);
        bottom.setInverted(false);
      } else {
        intake.setInverted(true);
        bottom.setInverted(true);
      }
    }
  }

  /**
   * Will pop from the queue
   * @return the top-most ball (the ball about to be shot)
   */
  public boolean eject()
  {
    if (cargo.size() == 0)
      return false;
    return cargo.poll();
  }

  /**
   * Should only be used when color sensor is not working
   * Used get/stop balls from getting in by pressing a joystick button
   */
  public void intake()
  {
    feed = !feed;
  }

  /**
   * Color sensor detects whether the color of the ball is our team color
   * Takes in color from sensor and checks it with the color previously 
   * inputted in SmartDashboard. The command will also return true if the color
   * sensor is disconnected, meaning that the driver can still shoot when color
   * sensor is no longer working. Regurgitation will only happen when the color
   * detected is incorrect. All the color of the balls are stored in the queue and
   * this method will insert values into the queue.
   * 
   * @return returns a boolean of whether or not the color is correct
   */
  public boolean detectColor() {

    // If the color sensor is disconnected, the driver still has the ability to shoot
    if (!m_colorSensor.isConnected())
      return true;
    
    Color detectedColor = m_colorSensor.getColor();
    char color = 'U';
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (m_colorSensor.getProximity() >= minProxmity) {
      if (match.color == Color.kBlue)
        color = 'B';
      else if (match.color == Color.kRed)
        color = 'R';
    } else {
      color = 'U';
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putString("Detected Color", Character.toString(color));
    if (color != 'U')
      cargo.add(color == teamColor);

    return color != 'U';
  }
}
