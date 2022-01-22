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
  String teamColor = SmartDashboard.getString("Team Color", "");

  /**
   * Color Sensor initialization
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  // The color sensor can detect how far away the object is from the sensor
  // This can be used to determine whether there is a ball or not
  private final int minProxmity = 2000;

  /** Creates a new ColorSensor. */
  public ColorSensor() {
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectColor();
  }

  public boolean detectColor() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     * 
     * @return returns a boolean of whether or not the color is correct
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString = "";
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
    
    if (m_colorSensor.getProximity() >= minProxmity) {
      if (match.color == Color.kBlue)
        colorString = "Blue";
      else if (match.color == Color.kRed)
        colorString = "Red";
    } else {
      colorString = "Unknown";
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
    SmartDashboard.putString("Detected Color", colorString);
    
    /**
     * Takes in color from sensor and checks it with the color previously 
     * inputted in SmartDashboard, if true, the detectColor method returns
     * true, if false, detectColor() returns false. Regurgitate will only run
     * if detectColor() returns false.
     * 
     * 
     * Can read through pexiglass
     * 
     * 
     */

    return teamColor.equals(colorString);
  }
}
