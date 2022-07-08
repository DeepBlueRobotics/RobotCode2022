// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private final double t = (BALL_VELOCITY_Y + Math.sqrt(BALL_VELOCITY_Y*BALL_VELOCITY_Y + 19.6*(SHOOTER_HEIGHT - GOAL_HEIGHT)))/9.8;
  private final TeleopDrive teleop;
  private final double speed = 0.1;
  
  public ShootMove(Drivetrain dt, Limelight limelight, TeleopDrive teleop) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt);
    this.limelight = limelight;
    this.teleop = teleop;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Orientation of dt is taken over to face a certain direction
    // shoot command will be parallelcommandgroup in robotcontainer

    double[] driverInputs = teleop.getAdjustedDriverInputs();
    // if limelight sees no goal
    // make robot turn at a constant velocity until it sees goal
    if(NetworkTableInstance.getDefault().getTable(limelight.config.ntName).getEntry("tv").getDouble(0.0) == 0) {
      dt.drive(driverInputs[0], driverInputs[1], limelight.config.steeringFactor * limelight.getIdleTurnDirection().sign);
      return;
    }
    double[] sides = limelight.determineObjectDist(CAMERA_HEIGHT, GOAL_HEIGHT, CAMERA_ANGLE);
    double dist = Math.hypot(sides[0], sides[1]);
    double odoRadian = dt.getOdometry().getPoseMeters().getRotation().getRadians(); // current angle of robot field-relative
    double limeRadian = Math.toRadians(NetworkTableInstance.getDefault().getTable(limelight.config.ntName).getEntry("tx").getDouble(0.0)); // goal-relative

    ChassisSpeeds speeds = dt.getSpeeds();

    double veloAngle = Math.atan2(-speeds.vyMetersPerSecond, speeds.vxMetersPerSecond); // robot-relative
    double velocity = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    double alpha = veloAngle + limeRadian;

    // the angle between the direction the robot is going
    double theta = dist * Math.sin(alpha) / (BALL_VELOCITY_X * t);

    // the angle that the robot should face field relative
    double fieldRelativeTheta = odoRadian - ( -theta - veloAngle );
    
    double robotTurn = 1;
    dt.drive(driverInputs[0], driverInputs[1], speed * (fieldRelativeTheta - odoRadian));

    SmartDashboard.putNumber("Goal Distance", dist);
    SmartDashboard.putNumber("Robot Relative Magnitude", velocity);
    /*
    SmartDashboard.putNumber("Field Relative Rotation", odoRadian);
    SmartDashboard.putNumber("Goal Relative Rotation", limeRadian);
    SmartDashboard.putNumber("Robot Relative Rotation", veloAngle);
    */
    SmartDashboard.putNumber("fieldRelativeTheta", fieldRelativeTheta);
    SmartDashboard.putNumber("Target Angle", fieldRelativeTheta - odoRadian);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
