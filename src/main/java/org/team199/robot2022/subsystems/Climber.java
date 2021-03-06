// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.team199.robot2022.Constants;

public class Climber extends SubsystemBase {

    private static final double kNEOFreeSpeedRPM = 5680;
    private static final double kDiameterIn = 1;
    private static final double kDesiredRetractSpeedInps = 1;
    private static final double kDesiredExtendSpeedInps = 6;

    // Torque is 2 * 9 * 0.5 = 9
    // Torque on the motor is Torque / ( gearing = 9 ) = 1

    private static final double kVoltsToCounterTorque = 10.5;

    private static final boolean leftInverted = false;

    private static final double extendPositionLeft = 5.317;
    private static final double extendPositionRight = 5.315;

    private static final double retractPositionLeft = -1.151;
    private static final double retractPositionRight = -1.172;
    private static final double gearing = 9;
    private static final double kInPerSec = ((kNEOFreeSpeedRPM / gearing) * Math.PI * kDiameterIn / 60);
    private static double kRetractSpeed = -((kDesiredRetractSpeedInps / kInPerSec)
            + (kVoltsToCounterTorque / 12)); // ~ -0.06151
    private static double kExtendSpeed = (kDesiredExtendSpeedInps / kInPerSec); // ~0.30261

    private static final double kSlowDesiredRetractSpeedInps = 1;
    private static final double kSlowDesiredExtendSpeedInps = 1;

    // Torque is 2 * 9 * 0.5 = 9
    // Torque on the motor is Torque / ( gearing = 9 ) = 1

    private static final double kSlowVoltsToCounterTorque = (1.1D / 32) * 12;
    private static final double kSlowRetractSpeed = -((kSlowDesiredRetractSpeedInps / kInPerSec)
            + (kSlowVoltsToCounterTorque / 12)); // ~ -0.06151
    private static final double kSlowExtendSpeed = (kSlowDesiredExtendSpeedInps / kInPerSec); // ~0.30261

    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();

    public Climber() {
        left.setInverted(leftInverted);
        right.setInverted(!leftInverted);

        leftEncoder.setPositionConversionFactor(1 / gearing);
        leftEncoder.setPosition(0);
        rightEncoder.setPositionConversionFactor(1 / gearing);
        rightEncoder.setPosition(0);
        SmartDashboard.putString("Left climber is", "Stopped");
        SmartDashboard.putString("Right climber is", "Stopped");
        SmartDashboard.putNumber("kDesiredExtendSpeedInps", kDesiredExtendSpeedInps);
        SmartDashboard.putNumber("kDesiredRetractSpeedInps", kDesiredRetractSpeedInps);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Climber Position", getLeftPosition());
        SmartDashboard.putNumber("Right Climber Position", getRightPosition());
    }

    public void resetEncodersToExtended() {
        leftEncoder.setPosition(extendPositionLeft);
        rightEncoder.setPosition(extendPositionRight);
    }

    public void resetEncodersToRetracted() {
        leftEncoder.setPosition(retractPositionLeft);
        rightEncoder.setPosition(retractPositionRight);
    }
    public void resetEncodersToZero() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void extendLeft() {
        left.set(kExtendSpeed);
        SmartDashboard.putString("Left climber is", "Extending");
    }

    public void extendRight() {
        right.set(kExtendSpeed);
        SmartDashboard.putString("Right climber is", "Extending");
    }

    public void retractLeft() {
        left.set(kRetractSpeed);
        SmartDashboard.putString("Left climber is", "Retracting");
    }

    public void retractRight() {
        right.set(kRetractSpeed);
        SmartDashboard.putString("Right climber is", "Retracting");
    }

    public void slowExtendLeft() {
        left.set(kSlowExtendSpeed);
        SmartDashboard.putString("Left climber is", "Extending");
    }

    public void slowExtendRight() {
        right.set(kSlowExtendSpeed);
        SmartDashboard.putString("Right climber is", "Extending");
    }
    public void slowRetractLeft() {
        left.set(kSlowRetractSpeed);
        SmartDashboard.putString("Left climber is", "Retracting");
    }

    public void slowRetractRight() {
        right.set(kSlowRetractSpeed);
        SmartDashboard.putString("Right climber is", "Retracting");
    }

    public void stop() {
        left.set(0);
        right.set(0);
        SmartDashboard.putString("Left climber is", "Stopped");
        SmartDashboard.putString("Right climber is", "Stopped");
    }

    public void stopLeft() {
        left.set(0);
        SmartDashboard.putString("Left climber is", "Stopped");
    }

    public void stopRight() {
        right.set(0);
        SmartDashboard.putString("Right climber is", "Stopped");
    }

    public void stop(boolean interrupted) {
        stop();
    }

    public void stopLeft(boolean interrupted) {
        stopLeft();
    }

    public void stopRight(boolean interrupted) {
        stopRight();
    }

    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    public boolean isLeftExtended() {
        return getLeftPosition() >= extendPositionLeft;
    }

    public boolean isRightExtended() {
        return getRightPosition() >= extendPositionRight;
    }

    public boolean isLeftRetracted() {
        return getLeftPosition() <= retractPositionLeft;
    }

    public boolean isRightRetracted() {
        return getRightPosition() <= retractPositionRight;
    }

    public boolean isRightResetExtended() {
        return getRightPosition() >= 0;
    }

    public boolean isLeftResetExtended() {
        return getRightPosition() >= 0;
    }

    public boolean isRightResetRetracted() {
        return getRightPosition() <= 0;
    }

    public boolean isLeftResetRetracted() {
        return getRightPosition() <= 0;
    }

}
