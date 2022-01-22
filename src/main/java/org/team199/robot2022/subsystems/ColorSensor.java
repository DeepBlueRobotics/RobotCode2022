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
   * Currently, there might be some errors with the I2C port but it shouldn't be an issue
   * as they probably will fix it before competition
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   * The example code that was copy pasted doesn't work kekw
   */
  //private final Color kBlueTarget = Color.makeColor(0.143, 0.427, 0.429);

  /**
   * may want to add WHAT COLOR BALLS THE ROBOT WANTS TO HAVE
   * change the boolean output for false to match the correct color
   */
  
   
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

  public void detectColor() {
    /**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     *
     * once the color is read, it will output to the smartdashboard
     * we can then use this information and put it into regurgitate
     * 
     * - create a boolean that detects if a ball is "correct", this boolean 
     * can then be switched around depending on what team we are.
     *    - ideally, we want the ball to be instantly regurgitated if it is not
     *      the "correct" color
     *    
     * 
     * 
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == Color.kBlue) {
      colorString = "Blue";
    } else if (match.color == Color.kRed) {
      colorString = "Red";
    } else {
      colorString = "Unknown";
    }

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the 
     * sensor.
     */
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
}
