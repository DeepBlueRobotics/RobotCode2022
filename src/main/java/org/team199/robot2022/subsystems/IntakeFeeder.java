package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

import java.util.Deque;
import java.util.LinkedList;

public class IntakeFeeder extends SubsystemBase {

  /**
   * Adds a box to SmartDashboard to put in team color
   * takes in what color the team is from smart dashboard to be checked
   * with color sensor
   */
  SendableChooser<Character> color = new SendableChooser<>();
  char teamColor;
  
  /** The color sensor can detect how far away the object is from the sensor
   *  This can be used to determine whether there is a ball or not
   *  This ranges from 0 to 2047 where the value is larger when an object is closer
   *  9.75 in
   */
  private final int minProxmity = 130; // TODO : Accurately determine minProxmity constant

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final CANSparkMax bottom = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeBottom); //TODO: set port
  private final CANSparkMax middle = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeMiddle);
  private final CANSparkMax top = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeTop); //TODO: set port

  private final double speed = 1.0;

  private boolean hasDetectedBall = false;
  // If there is a jam or carpet rolled over color sensor, override the color sensor's actions
  private boolean overrideSensor = false;

  /* Concern:
   * Make sure that when the ball is going thru the feeder that there is enough space between the balls
   * DO NOT let them rub against each other otherwise a jam can occur
   */

  // Will store what color balls are in the feeder
  // true = team color, false = not team color
  private Deque<Boolean> cargo = new LinkedList<>();

  // Should the robot intake balls or not (should only be used when color sensor is not working)
  // 0 = no balls, 1 = 1 ball, 2 = 2 balls
  // THIS IS ONLY FOR INTAKE NOT THE SHOOTER
  private int feed = 0;

  // if someone put the motor the wrong direction I don't have to manually switch the trues and falses
  boolean inverted = true;

  public IntakeFeeder() {
    if (!m_colorSensor.isConnected())
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);

    bottom.set(speed);

    color.setDefaultOption("Blue", 'B');
    color.addOption("Red", 'R');
    SmartDashboard.putData(color);
    SmartDashboard.putString("Add Ball to Queue", "");
    SmartDashboard.putNumber("Remove Ball from Queue", 0);
  }
  
  @Override
  public void periodic() {
    // Ocassionally update the team color if the team put the wrong one by accident
    teamColor = color.getSelected();

    if (m_colorSensor.isConnected() && !overrideSensor) 
      autonomousPeriodic();
    else 
      manualPeriodic();
    
    addBalls();
    debug();
  }

  /**
   * Run in the periodic method if color sensor is working
   */
  public void autonomousPeriodic()
  {
    feed = cargo.size();
    if (cargo.size() < 2)
    {
      bottom.set(speed);
    }

    if (detectColor()) {
      if (cargo.size() > 0 && cargo.peekFirst() == false)
      {
        middle.set(0);
        // Regurgitate via intake
        if (isBallThere(bottom)) {
          bottom.setInverted(inverted);
          bottom.set(speed);
        } else {
          bottom.setInverted(!inverted);
          bottom.set(speed);
          cargo.removeFirst();
        }
      }
      else if (cargo.size() == 1) {
        // while ball is still in color sensor range move the ball out to prevent jam
        // TODO : The ball might not reach the destination fast enough if second ball gets in
        if (!isBallThere(middle) && isBallThere(top))
        {
          middle.set(0);
          top.set(0);
        } else {
          middle.set(speed);
          top.set(speed);
        }
      }
      // If this ball is the second ball in the feeder
      else {
        // This is to prevent any more balls getting in
        top.set(0);
        middle.set(0);
        bottom.set(0);
        bottom.setInverted(!inverted);
      } 
    }
  }

  /**
   * Run only if color sensor is not working in replacement of the automous periodic method
   */
  public void manualPeriodic()
  {
    SmartDashboard.putString("Detected Color", "Disconnected");
    // Reset the balls in the cargo as color sensor no longer works and we cannot accurately record the cargo
    cargo = new LinkedList<>();
    // Manually intake balls
    switch(feed)
    {
      case 0:
        bottom.set(speed);
        bottom.setInverted(!inverted);
        middle.set(0);
        break;
      case 1:
        bottom.set(speed);
        bottom.setInverted(!inverted);
        middle.set(speed);
        break;
      case 2:
        bottom.set(0);
        middle.set(0);
        break;
    }
  }

  /**
   * Will pop from the queue
   * @return the top-most ball (the ball about to be shot)
   */
  public boolean eject() // TODO : Unfinished, need to integrate with shooter subsystem
  {
    if (cargo.size() == 0)
      return false;
    top.set(1);
    return cargo.pollFirst();
  }

  /**
   * Should only be used when color sensor is not working
   * Used get/stop balls from getting in by pressing a joystick button
   * by changing the feed variable which controls motors
   */
  public void manualAdd()
  {
    if (m_colorSensor.isConnected())
    {
      System.err.println("Color sensor is connected");
      return;
    }
    if (feed == 2) {
      System.err.println("You can't add any more balls!");
      return;
    }
    ++feed;
  }

  public void manualSub()
  {
    if (m_colorSensor.isConnected())
    {
      System.err.println("Color sensor is connected");
      return;
    }
    if (feed == 0) {
      System.err.println("You can't subtract any more balls!");
      return;
    }
    --feed;
  }

  /**
   * Regurgitates out of intake not shooter
   * Only used when color sensor no work
   * Does not automatically remove the ball from deque
   */
  public void regurgitate()
  {
    while(isBallThere(bottom)) 
    {  
      bottom.setInverted(inverted);
      bottom.set(speed);
    }
  }

  /**
   * Switches on or off to override sensor
   */
  public void override()
  {
    overrideSensor = !overrideSensor;
  }

  public boolean isBallThere(CANSparkMax motor) {
    return (motor.getEncoder().getVelocity() < (speed));
  }
  
  /**
   * Puts whether the ball is the team color or not and whether its in the feeder or shooter
   * in "shooter" basically means the next ball to be shot
   */
  public void debug()
  {
    Object[] arr = cargo.toArray();

    if (cargo.size() >= 2)
    {
      SmartDashboard.putString("Lower Ball", ((Boolean) arr[1]).toString());
    }
    else
    {
      SmartDashboard.putString("Lower Ball", "None");
    }

    if (cargo.size() >= 1)
    {
      SmartDashboard.putString("Upper Ball", ((Boolean) arr[0]).toString());
    }
    else
    {
      SmartDashboard.putString("Upper Ball", "None");
    }

    SmartDashboard.putNumber("Size", feed);
    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
  }

  /**
   * Another debugging tool to add/remove balls to the cargo
   */
  public void addBalls()
  {
    if (cargo.size() >= 2)
    {
      return;
    }
    else
    {
      char[] ballAddedArr = SmartDashboard.getString("Add Ball to Queue", "").toUpperCase().toCharArray();
      // By default set to "Unknown"
      char ballAdded = 'U';
      if (ballAddedArr.length > 0)
        ballAdded = ballAddedArr[0];
      int numBallsRemoved = (int)SmartDashboard.getNumber("Remove Ball(s) from Queue", 0);

      //REMOVES BALLS
      for (int i = 0; i < numBallsRemoved; i++)
      {
        cargo.pollFirst();
      }

      //ADDS BALLS
      if (ballAdded == 'T')
      {
        cargo.add(true);
      }
      else if (ballAdded == 'F')
      {
        cargo.add(false);
      }
    }
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
    if (!m_colorSensor.isConnected()) {
      hasDetectedBall = false;
      return true;
    }
    
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

    SmartDashboard.putString("Detected Color", Character.toString(color));
    if (color != 'U' && !hasDetectedBall) {
      cargo.addLast(color == teamColor);
      hasDetectedBall = true;
    }
    else if (color == 'U')
      hasDetectedBall = false;

    return color != 'U';
  }
}
