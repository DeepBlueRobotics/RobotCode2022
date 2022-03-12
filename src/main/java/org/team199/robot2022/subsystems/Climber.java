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

    private static final double kRetractSpeed = 0.4;
    private static final double kExtendSpeed = -0.1;

    private static final boolean leftInverted = true;

    private static final double extendPosition = 6;
    private static final double retractPosition = -1;
    private static final double gearing = 9;

    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);
    private final RelativeEncoder encoder = left.getEncoder();

    public Climber() {
        left.setInverted(leftInverted);
        right.follow(left, true);

        encoder.setPositionConversionFactor(1 / gearing);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Position", getPosition());
    }

    public void extend() {
        left.set(kExtendSpeed);
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);
    }

    public void retract() {
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
        left.set(kRetractSpeed);
    }

    public void stop() {
        left.setIdleMode(IdleMode.kBrake);
        right.setIdleMode(IdleMode.kBrake);
        left.set(0);
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
