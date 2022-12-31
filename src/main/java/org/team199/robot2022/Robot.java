// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022;

import org.carlmontrobotics.lib199.MotorErrors;
import org.carlmontrobotics.lib199.sim.MockedSparkEncoder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer(this);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double batteryVolts = robotContainer.pdp.getVoltage();
    double totalAmps = robotContainer.pdp.getTotalCurrent();
    SmartDashboard.putNumber("PDP Voltage", batteryVolts);
    SmartDashboard.putNumber("PDP Current", totalAmps);
    MotorErrors.printSparkMaxErrorMessages();
  }

  @Override
  public void simulationInit() {
    MockedSparkEncoder.setGearing(Constants.DrivePorts.driveFrontLeft, Constants.DriveConstants.driveGearing);
    MockedSparkEncoder.setGearing(Constants.DrivePorts.driveFrontRight, Constants.DriveConstants.driveGearing);
    MockedSparkEncoder.setGearing(Constants.DrivePorts.driveBackLeft, Constants.DriveConstants.driveGearing);
    MockedSparkEncoder.setGearing(Constants.DrivePorts.driveBackRight, Constants.DriveConstants.driveGearing);
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        robotContainer.dt.coast();
      } catch(InterruptedException e) {}
    }).start();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    robotContainer.dt.brake();
    robotContainer.dt.resetOdometry();
    robotContainer.getAutonomousCommand().schedule();
    robotContainer.initShooterConfig();
    robotContainer.setLimelightColor();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    CommandScheduler.getInstance().cancelAll();
    robotContainer.dt.brake();
    robotContainer.initShooterConfig();
    robotContainer.setLimelightColor();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    robotContainer.setLimelightColor();
  }

}
