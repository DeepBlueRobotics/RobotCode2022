package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.REVLibError;
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

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final CANSparkMax bottom = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeBottom); //TODO: set ports for motors
  private final CANSparkMax middle = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeMiddle);
  private final CANSparkMax top = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeTop);

  // Constant values that can be tweaked
  private double topSpeed = .333;
  private double midSpeed = .333;
  private double botSpeed = .333;
  // Used to calculate whether there is a ball against the motor
  private final double ampsThreshold = 4; // TODO : Get the best threshold that includes deflated balls
  private final int minProxmity = 400; // TODO : Accurately determine minProxmity constant

  private boolean hasDetectedBall = false;
  private boolean initRunMid = true;
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
  boolean botInverted = false;

  private Timer timer = new Timer();
  // How many seconds need to pass to determine whether something is jammed
  private final int jamThreshold = 5;

  public IntakeFeeder() {
    if (!m_colorSensor.isConnected())
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);

    bottom.set(botSpeed);

    color.setDefaultOption("Blue", 'B');
    color.addOption("Red", 'R');
    SmartDashboard.putData(color);
    SmartDashboard.putString("Add Ball to Queue", "");
    SmartDashboard.putNumber("Remove Ball from Queue", 0);
    SmartDashboard.putNumber("Size", feed);
    SmartDashboard.putNumber("Top Voltage", topSpeed);
    SmartDashboard.putNumber("Mid Voltage", midSpeed);
    SmartDashboard.putNumber("Bot Voltage", botSpeed);
  }
  
  @Override
  public void periodic() {
    // Ocassionally update the team color if the team put the wrong one by accident
    teamColor = color.getSelected();
    topSpeed = SmartDashboard.getNumber("Top Voltage", topSpeed);
    midSpeed = SmartDashboard.getNumber("Mid Voltage", midSpeed);
    botSpeed = SmartDashboard.getNumber("Bot Voltage", botSpeed);

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
      bottom.set(botSpeed);
    }

    if (cargo.size() > 0 && cargo.peekFirst() == false)
    {
      middle.set(0);
      // Regurgitate via intake
      if (!isJammed(bottom) && isBallThere(bottom)) {
        bottom.setInverted(botInverted);
        bottom.set(botSpeed);
      } else {
        // TODO : cargo.removeFirst() may remove the red ball but the color sensor will not detect that the ball is not
        // regurgitated and will not add the ball
        cargo.removeFirst();
        bottom.setInverted(!botInverted);
        bottom.set(botSpeed);
      }
    }
    if (detectColor()) {
      if (cargo.size() == 1 && cargo.peekFirst()) {
        // while ball is still in color sensor range move the ball out to prevent jam
        // TODO : The ball might not reach the destination fast enough if second ball gets in
        if ((!isBallThere(middle) && isBallThere(top)) || (middle.get() == 0 && top.get() == 0 && !initRunMid)) // TODO : Fix this, logic is wrong
        {
            middle.set(0);
            top.set(0);
        } 
        else {
          initRunMid = false;
          if (!isJammed(middle))
          {
            middle.set(midSpeed);
            top.set(topSpeed);
          }
          else if(isBallThere(middle))
          {
            unJam(middle, midSpeed);
          }
        }
      }
      // If this ball is the second ball in the feeder
      else {
        // This is to prevent any more balls getting in
        top.set(0);
        middle.set(0);
        bottom.set(0);
        bottom.setInverted(!botInverted);
      } 
    }
  }

  /**
   * Run only if color sensor is not working in replacement of the automous periodic method
   */
  public void manualPeriodic() // TODO : Need to convert all this into a command class to not interfere with shooter
  {
    feed = (int) SmartDashboard.getNumber("Size", feed);
    SmartDashboard.putString("Detected Color", "Disconnected");
    // Reset the balls in the cargo as color sensor no longer works and we cannot accurately record the cargo
    cargo = new LinkedList<>();
    // Manually intake balls
    switch(feed)
    {
      case 0:
        bottom.setInverted(!botInverted);
        middle.set(0);
        // if (!isJammed(bottom))
        // {
          bottom.set(botSpeed);
        // }
        // else
        // {
        //   unJam(bottom, botSpeed);
        // }
        break;
      case 1:
        bottom.setInverted(!botInverted);
        // if (!isJammed(bottom))
        // {
          bottom.set(botSpeed);
        // }
        // else
        // {
        //   unJam(bottom, botSpeed);
        // }
        
        if ((!isBallThere(middle) && isBallThere(top)) || (middle.get() == 0 && top.get() == 0 && !initRunMid))
        {
          middle.set(0);
          top.set(0);
        } else {
          initRunMid = false;
          // if (!isJammed(middle)) {
            middle.set(midSpeed);
            top.set(topSpeed);
          // }
          // else if(isJammed(middle))
          // {
          //   unJam(middle, midSpeed);
          // }
        }
        break;
      case 2:
        bottom.set(0);
        middle.set(0);
        top.set(0);
        bottom.setInverted(!botInverted);
        break;
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
    top.setInverted(!inverted);
    while (isBallThere(top) && !isJammed(top))
      top.set(topSpeed);
    
    if (isBallThere(top))
    {
      timer.reset();
      timer.start();
      // Tries to unjam by running motors opposite direction for one second
      while (timer.get() <= 1)
        top.setInverted(inverted);
        top.setInverted(!inverted);
        top.set(0);
      return false;
    }
    top.set(0);
    initRunMid = true;
    return cargo.pollFirst();
  }

  /**
   * Should only be used when color sensor is not working
   * Used get/stop balls from getting in by pressing a joystick button
   * by changing the feed variable which controls motors
   */
  public void manualAdd()
  {
    if (m_colorSensor.isConnected() && !overrideSensor)
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
    if (m_colorSensor.isConnected() && !overrideSensor)
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
    middle.set(0);
    while(isBallThere(bottom)) 
    {  
      bottom.setInverted(botInverted);
      bottom.set(botSpeed);
    }
    bottom.setInverted(!botInverted);
    bottom.set(botSpeed);
  }

  /**
   * Switches on or off to override sensor
   */
  public void override()
  {
    overrideSensor = !overrideSensor;
  }

  /**
   * @param motor
   * @return whether ball is there
   */
  public boolean isBallThere(CANSparkMax motor) {
    return motor.getOutputCurrent() > ampsThreshold;
  }
  
  /**
   * If motor is still trying to get a ball out for more than the number of seconds
   * stated in jamThreshold constant, then the ball is jammed
   * @param motor
   * @return
   */
  public boolean isJammed(CANSparkMax motor)
  {
    if (timer.get() == 0) {
      timer.start();
    }
    if (!isBallThere(motor))
    {
      timer.stop();
      timer.reset();
      return false;
    }
    if ((timer.get() > jamThreshold && isBallThere(motor)) || motor.getLastError() == REVLibError.kOk)
    {
      timer.stop();
      timer.reset();
      return true;
    }
    return false;
  }

  public void unJam(CANSparkMax motor, double speed)
  {
    if (timer.get() == 0) {
      timer.reset();
      timer.start();
    }
    if(timer.get() < 1)
    {
      motor.setInverted(inverted);
      motor.set(speed);
    }
    else
    {
      motor.setInverted(!inverted);
      motor.set(0);
      timer.stop();
      timer.reset();
    }
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

    SmartDashboard.putNumber("Proximity", m_colorSensor.getProximity());
    SmartDashboard.putNumber("Current", bottom.getOutputCurrent());
    SmartDashboard.putNumber("Top current", top.getOutputCurrent());
    SmartDashboard.putNumber("Middle current", middle.getOutputCurrent());
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
