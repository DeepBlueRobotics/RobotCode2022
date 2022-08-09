// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;

public class ShootMove extends CommandBase {
  /** Creates a new AutonomousShoot. */
  private Drivetrain dt;
  private Limelight limelight;
  private final double CAMERA_HEIGHT = 1.1519662;
  private final double CAMERA_ANGLE = 48; // in degrees
  // The goal (the very top) is 264cm tall
  private final double GOAL_HEIGHT = 2.64;
  private final double SHOOTER_HEIGHT = 1.144016;
  private final double BALL_VELOCITY_X = 2.31;
  private final double BALL_VELOCITY_Y = 6.52;
  // number of seconds ball is in air
  private final TeleopDrive teleop;
  private double turnAngle = Math.PI; // in radians
  private final double turnTolerance = 5 * Math.PI/180;

  private double kp = 0.1;
  private double min_turn = 2 * Math.PI/180;
  private double min_threshold = 5 * Math.PI/180;
  private final Timer timer;
  
  public ShootMove(Drivetrain dt, Limelight limelight, TeleopDrive teleop) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt);
    this.limelight = limelight;
    this.teleop = teleop;
    SmartDashboard.putNumber("ShootMove-kp", kp);
    //SmartDashboard.putNumber("min_turn", min_turn);
    //SmartDashboard.putNumber("min_threshold", min_threshold);

    timer = new Timer();
    timer.start();
  }

  public boolean shouldShoot()
  {
    if (Math.abs(turnAngle) < turnTolerance) {
      SmartDashboard.putBoolean("Shooting", true);
    } else {
      SmartDashboard.putBoolean("Shooting", false);
    }

    return (Math.abs(turnAngle) < turnTolerance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Orientation of dt is taken over to face a certain direction
    // shoot command will be two commands in robotcontainer
    
    double[] driverInputs = teleop.getAdjustedDriverInputs();
    SmartDashboard.putNumber("Elapsed Time ShootMove", timer.get());
    // if limelight sees no goal
    // make robot turn at a constant velocity until it sees goal
    if(NetworkTableInstance.getDefault().getTable(limelight.config.ntName).getEntry("tv").getDouble(0.0) < 0.01) {
      /*
      SmartDashboard.putNumber("Relative x", -1);
      SmartDashboard.putNumber("Relative y", -1);
      SmartDashboard.putNumber("Turn Angle", turnAngle*180/Math.PI);
      SmartDashboard.putNumber("Turn Angle Velocity", limelight.config.steeringFactor * limelight.getIdleTurnDirection().sign);
      SmartDashboard.putNumber("Time", -1);
      SmartDashboard.putNumber("Driver Input 0", driverInputs[0]);
      SmartDashboard.putNumber("Driver Input 1", driverInputs[1]);
      */
      SmartDashboard.putBoolean("Target Found", false);
      turnAngle = Math.PI;
      
      dt.drive(driverInputs[0], driverInputs[1], limelight.config.steeringFactor * turnAngle);
      
      return;
    }

    ChassisSpeeds speeds = dt.getSpeeds();

    //double dist = limelight.determineObjectDist(CAMERA_HEIGHT, GOAL_HEIGHT, CAMERA_ANGLE);
    // In degrees
    //double txDeg = NetworkTableInstance.getDefault().getTable(limelight.config.ntName).getEntry("tx").getDouble(0.0);
    
    // Robot-Relative
    double relative_x = limelight.determineObjectDist(CAMERA_HEIGHT, GOAL_HEIGHT, CAMERA_ANGLE)[0];
    double relative_y = limelight.determineObjectDist(CAMERA_HEIGHT, GOAL_HEIGHT, CAMERA_ANGLE)[1];
    // If not then something is seriously wrong
    // relative_x is the "forward" distance the ball needs to travel, and at this point the goal is in range of limelight
    // if its negative than y and x are swapped
    //assert relative_x >= 0;
    /*
    double relative_x = dist * Math.cos(txDeg / 180 * Math.PI);
    double relative_y = dist * Math.sin(txDeg / 180 * Math.PI);

    */
    RobotInfo robotInfo = new RobotInfo(relative_x, relative_y, speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, BALL_VELOCITY_X);
    //double currAngle = dt.getOdometry().getPoseMeters().getRotation().getRadians(); // Field-relative
    //SmartDashboard.putNumber("Relative x", relative_x);
    //SmartDashboard.putNumber("Relative y", relative_y);
    
    /*
    SmartDashboard.putNumber("Driver Input 0", driverInputs[0]);
    SmartDashboard.putNumber("Driver Input 1", driverInputs[1]);
    */
    SmartDashboard.putBoolean("Target Found", true);

    double[] info = solve_2d(robotInfo); // in radians

    turnAngle = info[1];
    double time = info[0];
    
    SmartDashboard.putNumber("Turn Angle", turnAngle*180/Math.PI);
    kp = SmartDashboard.getNumber("ShootMove-kp", kp);
    //min_turn = SmartDashboard.getNumber("min_turn", min_turn);
    //min_threshold = SmartDashboard.getNumber("min_threshold", min_threshold);

    //if (Math.abs(turnAngle) > min_threshold) {
      dt.drive(driverInputs[0], driverInputs[1], kp * turnAngle);
      SmartDashboard.putNumber("Turn Angle Velocity", kp * turnAngle);    
    //}
    /*
    else {
      if (turnAngle < 0) {
        dt.drive(driverInputs[0], driverInputs[1], kp * turnAngle - min_turn);
        SmartDashboard.putNumber("Turn Angle Velocity", kp * turnAngle - min_turn);    

      }
      else if (turnAngle > 0) {
        dt.drive(driverInputs[0], driverInputs[1], kp * turnAngle + min_turn);
        SmartDashboard.putNumber("Turn Angle Velocity", kp * turnAngle + min_turn);    

      }
    }
    */



    /*
    if (Math.abs(turnAngle + 1.0) < 0.01) {
      dt.drive(driverInputs[0], driverInputs[1], 0);
    } else {
    }
    */
    
    shouldShoot();
  }

  // Rearrangement of the system into standard form
  public double f(RobotInfo info, double theta)
  {
    return info.relative_x*(info.vel_y + info.speed_p*Math.sin(theta)) - info.relative_y*(info.vel_x + info.speed_p*Math.cos(theta));
  }

  public double f_prime(RobotInfo info, double theta)
  {
    return info.relative_x*info.speed_p*Math.cos(theta) + info.relative_y*info.speed_p*Math.sin(theta);
  }

  public double[] solve_2d(RobotInfo info)
  {
    // the easiest way to ensure the solution is non-degenerate is to seed newton's method with the direction to the goal
    // TODO: adjust for robot speed. radial/tangential velocity?
    int NEWTON_STEPS = (int)SmartDashboard.getNumber("NewtonStep", 30);
    double guess = Math.atan2(info.relative_y, info.relative_x);
    for (int i = 0; i < NEWTON_STEPS; i++) {
      guess -= f(info, guess)/f_prime(info, guess);
    }
    guess %= (2*Math.PI);
    if (guess > Math.PI)
    {
      guess = -(2*Math.PI - guess);
    }
    double x_time = info.relative_x / (info.vel_x + info.speed_p * Math.cos(guess));
    double y_time = info.relative_y / (info.vel_y + info.speed_p * Math.sin(guess));

    // Check if t is valid
    /*
    if (x_time < 0 || y_time < 0) {
      DriverStation.reportError("x_time " + x_time, false);
      DriverStation.reportError("y_time " + y_time, false);
      //return new double[]{-1, -1};
    }
    if (Math.abs(x_time-y_time) > .1) {
      DriverStation.reportError("x_time " + x_time, false);
      DriverStation.reportError("y_time " + y_time, false);
      //return new double[]{-1, -1};
    }
    */
    return new double[]{NEWTON_STEPS, guess};
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }

  // This class is for convenience, so that you don't have to pass in 6 parameters
  private class RobotInfo {

    // The robot distance relative to the goal (Limelight Calculations)
    double relative_x; 
    double relative_y; 

    // The velocity of the robot (ChassisSpeeds)
    double vel_x; 
    double vel_y; 
    
    // Speed of the ball in the planar (x-y) dimension (Sen-ac)
    double speed_p; 

    // Angle of the robot/shooter on the x-y plane (What we are trying to calculate)
    //double theta;

    public RobotInfo(double r_x, double r_y, double v_x, double v_y, double s_p)
    {
      relative_x = r_x; relative_y = r_y; vel_x = v_x; vel_y = v_y; speed_p = s_p;
    }
  }
}
