/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.robot2022.commands;

import java.util.function.Supplier;

import org.carlmontrobotics.lib199.Limelight;
import org.team199.robot2022.Constants;
import org.team199.robot2022.subsystems.Drivetrain;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopDrive extends CommandBase {
  private static final double kSlowDriveSpeed = 0.25;
  private static final double kSlowDriveRotation = 0.30;
  private static final double kAlignMultiplier = 1D/3D;
  private static final double kAlignForward = 0.6;

  private final Drivetrain drivetrain;
  private Supplier<Double> fwd;
  private Supplier<Double> str;
  private Supplier<Double> rcw;
  private Supplier<Boolean> slow;
  private Supplier<Boolean> align;
  private Limelight lime;
  double currentForward = 0;
  double currentStrafe = 0;
  /**
   * Creates a new TeleopDrive.
   */
  public TeleopDrive(Drivetrain drivetrain, Supplier<Double> fwd, Supplier<Double> str, Supplier<Double> rcw, Supplier<Boolean> slow, Supplier<Boolean> align, Limelight lime) {
    addRequirements(this.drivetrain = drivetrain);
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
    this.slow = slow;
    this.align = align;
    this.lime = lime;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawForward, rawStrafe, rotateClockwise, deltaT;
    deltaT = .05;
    // Sets all values less than or equal to a very small value (determined by the idle joystick state) to zero.
    // Used to make sure that the robot does not try to change its angle unless it is moving,
    if (Math.abs(fwd.get()) <= Constants.OI.JOY_THRESH) rawForward = 0.0;
    else rawForward = Constants.DriveConstants.maxForward * fwd.get();
    if (Math.abs(str.get()) <= Constants.OI.JOY_THRESH) rawStrafe = 0.0;
    else rawStrafe = Constants.DriveConstants.maxStrafe * str.get();

    rotateClockwise = rcw.get();
    if (Math.abs(rotateClockwise) <= Constants.OI.JOY_THRESH) rotateClockwise = 0.0;
    else rotateClockwise = Constants.DriveConstants.maxRCW * rotateClockwise;
    //double currentForward = drivetrain.getSpeeds().vxMetersPerSecond;
    //double currentStrafe = -drivetrain.getSpeeds().vyMetersPerSecond;
    double targetAccelerationX = (rawForward - currentForward)/deltaT;
    double targetAccelerationY = (rawStrafe - currentStrafe)/deltaT;
    double accelerationMagnitude = Math.hypot(targetAccelerationX, targetAccelerationY);
    if (accelerationMagnitude >= Constants.DriveConstants.autoMaxAccelMps2) {
      targetAccelerationX *= Constants.DriveConstants.autoMaxAccelMps2/accelerationMagnitude;
      targetAccelerationY *= Constants.DriveConstants.autoMaxAccelMps2/accelerationMagnitude;
    }
    currentForward += targetAccelerationX*deltaT;
    currentStrafe += targetAccelerationY*deltaT;
    if (Math.abs(currentForward) <= Constants.OI.JOY_THRESH)
      currentForward = 0;
    if (Math.abs(currentStrafe) <= Constants.OI.JOY_THRESH)
      currentStrafe = 0;
    //SmartDashboard.putNumber("Forward (mps)", currentForward);
   // SmartDashboard.putNumber("Strafe (mps)", currentStrafe);
    double driveMultiplier = slow.get() ? kSlowDriveSpeed : 1;
    double rotationMultiplier = slow.get() ? kSlowDriveRotation : 0.55;
    if(align.get()) {
      currentForward = Constants.DriveConstants.maxSpeed * kAlignForward * NetworkTableInstance.getDefault().getTable(lime.config.ntName).getEntry("tv").getDouble(0.0);
      currentStrafe = 0;
      driveMultiplier = 1;
      rotateClockwise = lime.steeringAssist();
      rotationMultiplier = Constants.DriveConstants.maxRCW * kAlignMultiplier;
    }
    drivetrain.drive(currentForward * driveMultiplier, currentStrafe * driveMultiplier, rotateClockwise * rotationMultiplier);
  }

  /*
  private void autoAlign() {
    double adjustment;
    if (limelightMode == Limelight.Mode.DIST) {
        adjustment = lime.distanceAssist();
        drivetrain.tankDrive(adjustment, adjustment, false);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
      else if (limelightMode == Limelight.Mode.STEER) {
        adjustment = lime.steeringAssist();
        //final double[] charParams = drivetrain.characterizedDrive(adjustment, -adjustment);
        drivetrain.tankDrive(adjustment, -adjustment, false);
        if (lime.isAligned())  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      } else {
        final double[] params = lime.autoTarget();
        drivetrain.tankDrive(params[0], params[1], false);
        final double maxInput = Math.max(Math.abs(params[0]), Math.abs(params[1]));
        if (maxInput < minError)  {
          SmartDashboard.putBoolean("Finished Aligning", true);
        }
      }
    SmartDashboard.putNumber("Left Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Rate", drivetrain.getEncRate(Drivetrain.Side.RIGHT));
    SmartDashboard.putNumber("Left Encoder Distance", drivetrain.getEncPos(Drivetrain.Side.LEFT));
    SmartDashboard.putNumber("Right Encoder Distance", drivetrain.getEncPos(Drivetrain.Side.RIGHT));
  }
  */

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
