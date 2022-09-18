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


    private static final double extendLeft = 5.317;
    private static final double extendRight = 5.315;
    private static final double retractLeft = -1.151;
    private static final double retractRight = -1.172;
    private static final double zero = 0;

    public static enum EncoderPos{
      extendLeft,
      retractLeft,
      extendRight,
      retractRight,
      zero,
    };


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

    public static enum MotorSpeed{
      kSlowExtendSpeed,
      kSlowRetractSpeed,
      kExtendSpeed,
      kRetractSpeed,
    }

    private static final int leftMotor = -1;//because someone requested this and there's no point slapping them in an enum
    private static final int bothMotors = 0;
    private static final int rightMotor = 1;


    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();


    private final Consumer<void>[] setEncoder = {//faster to array[](inp) than a bunch of if's in a func
      (EncoderPos pos) -> leftEncoder.setPosition(pos),
      (EncoderPos pos) -> {leftEncoder.setPosition(pos);rightEncoder.setPosition(pos);},
      (EncoderPos pos) -> rightEncoder.setPosition(pos),
    };

    private final Consumer<void>[] setMotor = {
      (MotorSpeed speed) -> {left.set(speed);SmartDashboard.putString("Left Climber State", "Moving");},
      (MotorSpeed speed) -> {left.set(speed);right.set(speed);SmartDashboard.putString("Left Climber State", "Moving");SmartDashboard.putString("Right Climber State", "Moving");},
      (MotorSpeed speed) -> {right.set(speed);SmartDashboard.putString("Right Climber State", "Moving");},
    };

    private final Consumer<double>[] getMotorPos = {
      () -> leftEncoder.getPosition(),
      () -> rightEncoder.getPosition(),
    };


    public Climber() {
        left.setInverted(leftInverted);
        right.setInverted(!leftInverted);

        leftEncoder.setPositionConversionFactor(1 / gearing);
        leftEncoder.setPosition(0);
        rightEncoder.setPositionConversionFactor(1 / gearing);
        rightEncoder.setPosition(0);
        SmartDashboard.putString("Left Climber State", "Stop");
        SmartDashboard.putString("Right Climber State", "Stop");
        SmartDashboard.putNumber("kDesiredExtendSpeedInps", kDesiredExtendSpeedInps);
        SmartDashboard.putNumber("kDesiredRetractSpeedInps", kDesiredRetractSpeedInps);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("L Climber Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("R Climber Pos", rightEncoder.getPosition());
    }

    //motor = -1 left or 1 right or 0 both
    public void resetEncodersTo(EncoderPos pos,int motor){
			setEncoder[motor+1].apply(pos);
    }

    public void moveMotors(MotorSpeed speed, int motor){
      setMotor[motor+1].apply(speed);
    }

    public void stopMotors(int motor){
      if (motor<1){
        left.set(0);
        SmartDashboard.putString("Left climber is", "Stopped");
      }
      if (motor>-1){
        right.set(0);
        SmartDashboard.putString("Right climber is", "Stopped");
      }
    }


    public boolean isMotorExtended(int motor){//is the motor(s) extended?
      if (motor<1 && getMotorPos[0].apply() < EncoderPos.extendLeft){
        return false;
      }
      if (motor>-1 && getMotorPos[1].apply() < EncoderPos.extendRight){
        return false;
      }
      return true;
    }

    public boolean isMotorRetracted(int motor){//is the motor(s) retracted?
      if (motor<1 && getMotorPos[0].apply() > EncoderPos.retractLeft){
        return false;
      }
      if (motor>-1 && getMotorPos[1].apply() > EncoderPos.retractRight){
        return false;
      }
      return true;
    }

}
