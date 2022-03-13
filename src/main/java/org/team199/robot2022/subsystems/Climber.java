// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import org.team199.robot2022.Constants;

public class Climber extends SubsystemBase {

    private static final double kNEOFreeSpeedRPM = 5680;
    private static final double kDiameterIn = 1;
    private static final double kDesiredRetractSpeedInps = 1;
    private static final double kDesiredExtendSpeedInps = 1;

    // Torque is 2 * 9 * 0.5 = 9
    // Torque on the motor is Torque / ( gearing = 9 ) = 1

    private static final double kVoltsToCounterTorque = (1D / 32) * 12;

    private static final boolean leftInverted = false;

    private static final double extendPosition = 3.88;
    private static final double retractPosition = -0.6;
    private static final double gearing = 9;
    private static final double kInPerSec = ( (kNEOFreeSpeedRPM / gearing) * Math.PI * kDiameterIn / 60 );

    private static final double kRetractSpeed = -( ( kDesiredRetractSpeedInps / kInPerSec )  + ( kVoltsToCounterTorque / 12 ) ); // ~ -0.06151
    private static final double kExtendSpeed = ( kDesiredExtendSpeedInps / kInPerSec ); // ~0.30261

    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);
    private final RelativeEncoder encoder = right.getEncoder();

    public Climber() {
        left.setInverted(leftInverted);
        right.follow(left, true);

        encoder.setPositionConversionFactor(1 / gearing);
        encoder.setPosition(0);
        SmartDashboard.putString("Climber is", "Stopped");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    public void extend() {
        left.set(kExtendSpeed);
        SmartDashboard.putString("Climber is", "Extending");
        // left.setIdleMode(IdleMode.kCoast);
        // right.setIdleMode(IdleMode.kCoast);
    }

    public void retract() {
        // left.setIdleMode(IdleMode.kBrake);
        // right.setIdleMode(IdleMode.kBrake);
        left.set(kRetractSpeed);
        SmartDashboard.putString("Climber is", "Retracting");
    }

    public void stop() {
        // left.setIdleMode(IdleMode.kBrake);
        // right.setIdleMode(IdleMode.kBrake);
        left.set(0);
        SmartDashboard.putString("Climber is", "Stopped");
    }

    public void stop(boolean interrupted) {
        stop();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public boolean isExtended() {
        return getPosition() >= extendPosition;
    }

    public boolean isRetracted() {
        return getPosition() <= retractPosition;
    }

}
