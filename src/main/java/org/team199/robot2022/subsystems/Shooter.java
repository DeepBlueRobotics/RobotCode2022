package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;

import org.team199.robot2022.Constants;

//import org.team199.robot2021.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
//import frc.robot.lib.MotorControllerFactory;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import frc.robot.lib.logging.Log;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import java.util.LinkedList;
import java.util.Queue;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;

    private static final double regularShooterRPM = 3320; 

    private double kTargetSpeed = 60;
    private double kTopWheelSpeed = 0.1; //check this
    private final double speedOffsetMain = 100;

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterSlave);
    private final CANSparkMax topwheel = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterTop);
    private final SparkMaxPIDController pidController = master.getPIDController();

    // Adds a box to SmartDashboard, takes in team color from SmartDashboard to check it with color sensor
    SendableChooser<Character> color = new SendableChooser<>();

    // Team Color is blue by default
    private char teamColor = 'B';

    // TODO: not sure what we use this for for shooter yet
    private final int minProximity = 150;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private boolean hasDetectedBall = false;

    public Shooter() {
        master.setSmartCurrentLimit(40);
        slave.setSmartCurrentLimit(40);
        topwheel.setSmartCurrentLimit(40);
        SmartDashboard.putNumber("Shooter.kTargetSpeed", kTargetSpeed);
        SmartDashboard.putNumber("Shooter.kTopWheelSpeed",kTopWheelSpeed);
        SmartDashboard.putNumber("Shooter.kP", kP);
        SmartDashboard.putNumber("Shooter.kI", kI);
        SmartDashboard.putNumber("Shooter.kD", kD);
        SmartDashboard.putNumber("Shooter.kV", kV);
        SmartDashboard.putNumber("Shooter.kS", kS);
        
        if (!m_colorSensor.isConnected()){
            SmartDashboard.putString("Detected Color", "Error, the color sensor is disconnected");
        }
        m_colorMatcher.addColorMatch(Color.kBlue);
        m_colorMatcher.addColorMatch(Color.kRed);

        color.addOption("Red",'R');
        color.setDefaultOption("Blue",'B');

        slave.follow(master, true);
        master.setInverted(true);
        Log.registerDoubleVar("Spark Max Port 3 Speed", () -> master.getEncoder().getVelocity());
        Log.registerDoubleVar("Spark Max Port 4 Speed", () -> slave.getEncoder().getVelocity());
        Log.registerDoubleVar("Spark Max Port 5 Speed", () -> topwheel.getEncoder().getVelocity());
    }

    public void periodic()  {
        double p = SmartDashboard.getNumber("Shooter.kP", kP);
        double i = SmartDashboard.getNumber("Shooter.kI", kI);
        double d = SmartDashboard.getNumber("Shooter.kD", kD);

        kV = SmartDashboard.getNumber("Shooter.kV", kV);
        kS = SmartDashboard.getNumber("Shooter.kS", kS);
        setMainSpeed(SmartDashboard.getNumber("Shooter.kTargetSpeed", kTargetSpeed));
        setTopWheelSpeed(SmartDashboard.getNumber("Shooter.kTopWheelSpeed",kTopWheelSpeed));

        if (p != pidController.getP()) pidController.setP(p);
        if (i != pidController.getI()) pidController.setI(i);
        if (d != pidController.getD()) pidController.setD(d);
        pidController.setReference(getTargetSpeed(), ControlType.kVelocity, 0, calculateFeedForward(getTargetSpeed()));
        
        topwheel.set(kTopWheelSpeed);

        SmartDashboard.putNumber("Speed Spark Max Port 3", master.getEncoder().getVelocity());
        SmartDashboard.putNumber("Speed Spark Max Port 4", slave.getEncoder().getVelocity());
        SmartDashboard.putNumber("Speed Spark Max Port 5", topwheel.getEncoder().getVelocity());
    }

    public void setMainSpeed(double 
    mainSpeed) {
        kTargetSpeed = mainSpeed;
    }

    public void setTopWheelSpeed(double topWheelSpeed){
        kTopWheelSpeed = topWheelSpeed;
    }

    public double getTargetSpeed() {
        return kTargetSpeed;
    }

    public double getTopWheelSpeed() {
        return kTopWheelSpeed;
    }

    public boolean isAtTargetSpeed() {
        return (master.getEncoder().getVelocity() > (kTargetSpeed - speedOffsetMain));
    }

    public double calculateFeedForward(double velocity) {
        return kS * Math.signum(velocity) + kV * velocity;
    }

    //if motor velocity is slower than usual, returns a boolean
    public boolean isBallThere() {
        return (master.getEncoder().getVelocity() < (regularShooterRPM));
    }

}
