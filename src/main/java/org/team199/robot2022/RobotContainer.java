// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022;

import org.team199.robot2022.commands.TeleopDrive;
import org.team199.robot2022.subsystems.Drivetrain;
import org.team199.robot2022.subsystems.Shooter;
import org.team199.robot2022.subsystems.ColorSensor;

import java.io.IOException;

import org.team199.robot2022.commands.Autonomous;
import org.team199.robot2022.commands.Shoot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.lib.path.RobotPath;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final Joystick leftJoy = new Joystick(Constants.OI.LeftJoy.port);
  public final Joystick rightJoy = new Joystick(Constants.OI.RightJoy.port);
  public final Joystick controller = Constants.OI.Controller.controller;

  public final Drivetrain dt = new Drivetrain();
  public final PowerDistribution pdp = new PowerDistribution();
  public final Shooter shooter = new Shooter();

  public final ColorSensor colorSensor = new ColorSensor();

  public final DigitalInput[] autoSelectors;
  public final AutoPath[] autoPaths;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    autoPaths = new AutoPath[] {
      new AutoPath(false, loadPath("Taxi1"), null, false, false),
      new AutoPath(false, loadPath("Taxi2"), null, false, false),
      new AutoPath(true, loadPath("Path1(1)"), loadPath("Path1(2)"), true, true),
      new AutoPath(true, loadPath("Path2(1)"), loadPath("Path2(2)"), true, true),
      new AutoPath(true, loadPath("Path3(1)"), loadPath("Path3(2)"), true, true)
    };

    autoSelectors = new DigitalInput[Math.min(autoPaths.length, 10)];
    for(int i = 0; i < autoSelectors.length; i++) {
      autoSelectors[i] = new DigitalInput(i);
    }

    if (DriverStation.isJoystickConnected(Constants.OI.LeftJoy.port)) {
      configureButtonBindingsLeftJoy();
    } else {
      System.err.println("ERROR: Dude, you're missing the left joystick.");
    }

    if (DriverStation.isJoystickConnected(Constants.OI.RightJoy.port)) {
      configureButtonBindingsRightJoy();
    } else {
      System.err.println("ERROR: Dude, you're missing the right joystick.");
    }

    if (DriverStation.isJoystickConnected(Constants.OI.Controller.port)) {
      configureButtonBindingsController();
    } else {
      System.err.println("ERROR: Dude, you're missing the controller.");
    }

    dt.setDefaultCommand(new TeleopDrive(dt,
        () -> inputProcessing(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.Y)),
        () -> inputProcessing(getStickValue(Constants.OI.StickType.LEFT, Constants.OI.StickDirection.X)),
        () -> inputProcessing(getStickValue(Constants.OI.StickType.RIGHT, Constants.OI.StickDirection.X)), () -> leftJoy.getRawButton(1) || rightJoy.getRawButton(1)));
  }

  private void configureButtonBindingsLeftJoy() {

  }

  private void configureButtonBindingsRightJoy() {
    new JoystickButton(rightJoy, Constants.OI.RightJoy.shootPort).whileHeld(new Shoot(colorSensor, shooter));
  }

  private void configureButtonBindingsController() {

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    AutoPath path = getAutoPath();
    return path == null ? new InstantCommand() : new Autonomous(path, dt, colorSensor,/*intakeFeeder,*/ shooter);
  }

  private double getStickValue(Constants.OI.StickType stick, Constants.OI.StickDirection dir) {
    switch (Constants.OI.CONTROL_TYPE) {
      case JOYSTICKS:
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X)
          return leftJoy.getX();
        if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y)
          return -leftJoy.getY();
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X)
          return rightJoy.getX();
        if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y)
          return -rightJoy.getY();
      case GAMEPAD:
        if (controller.getName().equals("Logitech Dual Action")) {
          if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X)
            return controller.getRawAxis(0);
          if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y)
            return -controller.getRawAxis(1);
          if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X)
            return controller.getRawAxis(2);
          if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y)
            return -controller.getRawAxis(3);
        } else {
          if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.X)
            return controller.getRawAxis(0);
          if (stick == Constants.OI.StickType.LEFT && dir == Constants.OI.StickDirection.Y)
            return -controller.getRawAxis(1);
          if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.X)
            return controller.getRawAxis(4);
          if (stick == Constants.OI.StickType.RIGHT && dir == Constants.OI.StickDirection.Y)
            return -controller.getRawAxis(5);
        }
      default:
        return 0;
    }
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1
   * 
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }

  public AutoPath getAutoPath() {
    for(int i = 0; i < autoSelectors.length; i++) {
      if(autoSelectors[i].get())
        return autoPaths[i];
    }
    return null;
  }

  public RobotPath loadPath(String pathName) {
    try {
      return new RobotPath(pathName, dt, false, new Translation2d());
    } catch(IOException e) {
      e.printStackTrace();
      return null;
    }
  }
}
