package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import org.team199.robot2022.Constants;
import org.team199.robot2022.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.SparkVelocityPIDController;
import frc.robot.lib.MotorErrors.TemperatureLimit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;
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
  Alliance teamColor;

  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();

  private final CANSparkMax bottom = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeBottom, TemperatureLimit.NEO); //TODO: set ports for motors
  private final CANSparkMax middle = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeMiddle, TemperatureLimit.NEO_550);
  private final CANSparkMax top = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kIntakeTop, TemperatureLimit.NEO_550);

  private final SparkVelocityPIDController middlePID;
  private final SparkVelocityPIDController topPID;

  // Constant values that can be tweaked
  private double topSpeed = 400;
  private double midSpeed = 400;
  private double botSpeed = 0.7;

  private double rpmTolerance = 7;
  // Used to calculate whether there is a ball against the motor
  private double minProxmity = 21.5; // TODO : Accurately determine minProxmity constant
  private double maxProxmity = 19.5; // TODO : Accurately determine minProxmity constant

  private boolean hasDetectedBall = false;
  // If there is a jam or carpet rolled over color sensor, override the color sensor's actions
  private boolean overrideSensor = false;
  private boolean dumbMode = false;

  private Alliance currentColor = Alliance.Invalid;
  private boolean ballDetected = false;

  /* Concern:
   * Make sure that when the ball is going thru the feeder that there is enough space between the balls
   * DO NOT let them rub against each other otherwise a jam can occur
   */

  // Will store what color balls are in the feeder
  // true = team color, false = not team color
  private Deque<Boolean> cargo = new LinkedList<>();

  // if someone put the motor the wrong direction I don't have to manually switch the trues and falses
  boolean botInverted = false;
  boolean midInverted = true;
  boolean topInverted = false;

  private static double proximityVal = 0;

  public IntakeFeeder(Robot robot) {
    if (!m_colorSensor.isConnected())
      SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
    m_colorMatcher.addColorMatch(Color.kBlue);
    m_colorMatcher.addColorMatch(Color.kRed);

    teamColor = DriverStation.getAlliance();

    bottom.setInverted(botInverted);
    middle.setInverted(midInverted);
    top.setInverted(topInverted);
    SmartDashboard.putNumber("Min Proximity", minProxmity);
    SmartDashboard.putNumber("Max Proximity", maxProxmity);
    SmartDashboard.putString("Add Ball to Queue", "");
    SmartDashboard.putNumber("Remove Ball from Queue", 0);
    SmartDashboard.putNumber("Size", 0);
    SmartDashboard.putNumber("Top Speed", topSpeed);
    SmartDashboard.putNumber("Mid Speed", midSpeed);
    SmartDashboard.putNumber("Bot Speed", botSpeed);
    SmartDashboard.putBoolean("IntakeFeeder Dumb Mode", isDumbModeEnabled());

    middlePID = new SparkVelocityPIDController("Intake Feeder (Middle)", middle, 0, 0, 0, 0, 0.0106, 0, rpmTolerance); //TODO: make sure feeder runs later
    topPID = new SparkVelocityPIDController("Intake Feeder (Top)", top, 0, 0, 0, 0, 0.0107, 0, rpmTolerance); //TODO: make sure feeder runs later

    middlePID.getEncoder().setVelocityConversionFactor(0.1);
    topPID.getEncoder().setVelocityConversionFactor(0.1);

    m_colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes16bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain3x);
    m_colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes8bit, ProximitySensorMeasurementRate.kProxRate6ms);

    cargo.add(true);
    robot.addPeriodic(this::updateColorSensor, 0.003);
  }

  @Override
  public void periodic() {
    // Ocassionally update the team color if the team put the wrong one by accident
    topSpeed = SmartDashboard.getNumber("Top Speed", topSpeed);
    midSpeed = SmartDashboard.getNumber("Mid Speed", midSpeed);
    botSpeed = SmartDashboard.getNumber("Bot Speed", botSpeed);
    SmartDashboard.putNumber("Top Actual Speed", top.getEncoder().getVelocity());
    SmartDashboard.putNumber("Mid Actual Speed", middle.getEncoder().getVelocity());
    SmartDashboard.putNumber("Bot Actual Speed", bottom.getEncoder().getVelocity());
    SmartDashboard.putNumber("Bot Current", bottom.getOutputCurrent());
    SmartDashboard.putNumber("Bot Temp", bottom.getMotorTemperature());
    SmartDashboard.putNumber("Bot Applied Voltage", bottom.getAppliedOutput());
    minProxmity = SmartDashboard.getNumber("Min Proximity", minProxmity);
    maxProxmity = SmartDashboard.getNumber("Max Proximity", maxProxmity);
    middlePID.periodic();
    topPID.periodic();

    SmartDashboard.putBoolean("IntakeFeeder Autonomous Control", useAutonomousControl());
    dumbMode = SmartDashboard.getBoolean("IntakeFeeder Dumb Mode", isDumbModeEnabled());

    SmartDashboard.putString("Detected Color", currentColor.toString());
    SmartDashboard.putNumber("Size", cargo.size());
    SmartDashboard.putNumber("Proximity", proximityVal);
    proximityVal = 0;

    addBalls();
    debug();
  }

  public void updateColorSensor() {
    if(!m_colorSensor.isConnected()) return;

    double proximity = m_colorSensor.getProximity();

    if(proximity >= minProxmity) ballDetected = true;
    else if(proximity < maxProxmity) ballDetected = false;

    Color detectedColor = m_colorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (ballDetected) {
      if (match.color == Color.kBlue)
        currentColor = Alliance.Blue;
      else if (match.color == Color.kRed)
        currentColor = Alliance.Red;
    } else {
      currentColor = Alliance.Invalid;
    }
    proximityVal = Math.max(proximityVal, proximity);
    if (!dumbMode || useAutonomousControl())
      detectColor();
  }

  public void toggleIntake() {
    invertAndRun(Motor.BOTTOM, false, bottom.get() == 0);
  }

  public boolean useAutonomousControl() {
    return m_colorSensor.isConnected() && !overrideSensor;
  }

  public Deque<Boolean> getCargo()
  {
    return cargo;
  }

  public void popFirstBall()
  {
    if (cargo.size() > 0)
      cargo.pollLast();
  }

  public void popSecondBall()
  {
    if (cargo.size() > 0)
      cargo.pollFirst();
  }

  public void clearCargo() {
    cargo.clear();
  }

  public int getNumBalls() {
    return cargo.size();
  }

  public void run(Motor motor, boolean running) {
    double speed = 0;
    if(running){
      switch(motor) {
        case BOTTOM:
          speed = botSpeed;
          break;
        case MIDDLE:
          speed = midSpeed;
          break;
        case TOP:
          speed = topSpeed;
          break;
      }
    }
    switch(motor) {
      case BOTTOM:
        bottom.set(speed);
        break;
      case MIDDLE:
        middlePID.setTargetSpeed(speed);
        break;
      case TOP:
        topPID.setTargetSpeed(speed);
        break;
    }
  }

  public void invert(Motor motor, boolean inverted) {
    boolean isMotorInverted = false;
    switch(motor) {
      case BOTTOM:
        isMotorInverted = botInverted;
        break;
      case MIDDLE:
        isMotorInverted = midInverted;
        break;
      case TOP:
        isMotorInverted = topInverted;
        break;
    }
    getMotor(motor).setInverted(isMotorInverted ? !inverted : inverted);
  }

  public void invertAndRun(Motor motor, boolean inverted, boolean isRunning) {
    invert(motor, inverted);
    run(motor, isRunning);
  }

  /**
   * Will pop from the queue
   * @return the top-most ball (the ball about to be shot)
   */
  public boolean eject()
  {
    if (cargo.size() == 0)
      return false;
    return cargo.pollFirst();
  }

  public void runForward() {
    //cargo.clear();
    bottom.setInverted(botInverted);
    middle.setInverted(midInverted);
    top.setInverted(topInverted);

    bottom.set(botSpeed);
    middlePID.setTargetSpeed(midSpeed);
    topPID.setTargetSpeed(topSpeed);
  }

  public void runBackward() {
    cargo.clear();
    bottom.setInverted(!botInverted);
    middle.setInverted(!midInverted);
    top.setInverted(!topInverted);

    bottom.set(botSpeed);
    middlePID.setTargetSpeed(midSpeed);
    topPID.setTargetSpeed(topSpeed);
  }
  public void stopRunningFeeder(){
    middlePID.setTargetSpeed(0);
    topPID.setTargetSpeed(0);
  }
  public void stop(){
    run(Motor.BOTTOM, false);
    run(Motor.MIDDLE, false);
    run(Motor.TOP, false);
  }

  /**
   * Should only be used when color sensor is not working
   * Used get/stop balls from getting in by pressing a joystick button
   * by changing the feed variable which controls motors
   */
  public void manualAdd()
  {
    /*
    if (m_colorSensor.isConnected() && !overrideSensor)
    {
      System.err.println("Color sensor is connected");
      return;
    }
    */
    if (cargo.size() == 2) {
      System.err.println("You can't add any more balls!");
      return;
    }

    cargo.addLast(true);
  }

  public void manualSub()
  {
    /*
    if (m_colorSensor.isConnected() && !overrideSensor)
    {
      System.err.println("Color sensor is connected");
      return;
    }
    */
    if (cargo.size() == 0) {
      System.err.println("You can't subtract any more balls!");
      return;
    }
    cargo.pollLast();
  }

  /**
   * Switches on or off to override sensor
   */
  public void override()
  {
    overrideSensor = !overrideSensor;
  }

  public void toggleDumbMode()
  {
    dumbMode = !dumbMode;
    if(!dumbMode) cargo.clear();
    SmartDashboard.putBoolean("IntakeFeeder Dumb Mode", isDumbModeEnabled());
  }

  public boolean isDumbModeEnabled()
  {
    return dumbMode;
  }

  public CANSparkMax getMotor(Motor motor) {
    switch(motor) {
      case BOTTOM:
        return bottom;
      case MIDDLE:
        return middle;
      case TOP:
        return top;
      default:
        return bottom;
    }
  }

  public boolean isBallThere(Motor motor) {
    switch(motor) {
      case MIDDLE:
        return !middlePID.isAtTargetSpeed();
      case TOP:
        return !topPID.isAtTargetSpeed();
      case BOTTOM:
        return m_colorSensor.isConnected() ? ballDetected : false;
      default:
        return false;
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

    SmartDashboard.putNumber("Bottom Current", bottom.getOutputCurrent());
    SmartDashboard.putNumber("Top current", top.getOutputCurrent());
    SmartDashboard.putNumber("Middle current", middle.getOutputCurrent());
    SmartDashboard.putString("Current Command (IntakeFeeder)", getCurrentCommand() == null ? "<None>" : getCurrentCommand().getName());
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
   */
  public void detectColor() {

    // If the color sensor is disconnected, the driver still has the ability to shoot
    if (!m_colorSensor.isConnected()) {
      hasDetectedBall = false;
      return;
    }

    if (ballDetected && !hasDetectedBall) {
      if (cargo.size() < 2) {
        cargo.addLast(currentColor == teamColor);
      }
      hasDetectedBall = true;
    }
    else if (!ballDetected)
      hasDetectedBall = false;
  }

  public static enum Motor {
    BOTTOM, MIDDLE, TOP;
  }

}
