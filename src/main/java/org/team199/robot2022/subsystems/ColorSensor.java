// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {

  /**
   * Adds a box to SmartDashboard to put in team color
   * takes in what color the team is from smart dashboard to be checked
   * with color sensor
   */
  boolean ignored = SmartDashboard.putString("Team Color", "");
  String teamColor = SmartDashboard.getString("Team Color", "").toUpperCase();

  /** The color sensor can detect how far away the object is from the sensor
   *  This can be used to determine whether there is a ball or not
   *  This ranges from 0 to 2047 where the value is larger when an object is closer
   */
  private final int minProxmity = 2000;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  public ColorSensor() {
    // If the color sensor does not respond
    if (!m_colorSensor.isConnected())
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);
  }

  @Override
  public void periodic() {
    // Ocassionally update the team color if the team put the wrong one by accident
    teamColor = SmartDashboard.getString("Team Color", "").toUpperCase();

    // Imperative to inform the driver whether color sensor is working
    if (m_colorSensor.isConnected())
      detectColor();
    else SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
  }

  /**
   * Color sensor detects whether the color of the ball is our team color
   * Takes in color from sensor and checks it with the color previously 
   * inputted in SmartDashboard. The command will also return true if the color
   * sensor is disconnected, meaning that the driver can still shoot when color
   * sensor is no longer working. Regurgitation will only happen when the color
   * detected is incorrect.
   * 
   * @return returns a boolean of whether or not the color is correct
   */
  public boolean detectColor() {

    // If the color sensor is disconnected, the driver still has the ability to shoot
    if (!m_colorSensor.isConnected())
      return true;
    
    Color detectedColor = m_colorSensor.getColor();
    String colorString = "";
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (m_colorSensor.getProximity() >= minProxmity) {
      if (match.color == Color.kBlue)
        colorString = "BLUE";
      else if (match.color == Color.kRed)
        colorString = "RED";
    } else {
      colorString = "UNKNOWN";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor. DEBUGGING STUFF NOT REALLY USEFUL IN GAME
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    // This is important
    SmartDashboard.putString("Detected Color", colorString);

    return teamColor.equals(colorString);
  }
}
