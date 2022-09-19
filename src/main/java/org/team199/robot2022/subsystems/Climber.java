// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.function.Consumer;
import java.util.Hashtable;
import java.util.Dictionary;

import org.team199.robot2022.Constants;

public class Climber extends SubsystemBase {

    private static final double kNEOFreeSpeedRPM = 5680;
    private static final double kDiameterIn = 1;
    private static final double kDesiredRetractSpeedInps = 1;
    private static final double kDesiredExtendSpeedInps = 6;
    private static final boolean leftInverted = false;

    private static final double gearing = 9;
    private static final double kInPerSec = ((kNEOFreeSpeedRPM / gearing) * Math.PI * kDiameterIn / 60);
    private static final double kSlowDesiredRetractSpeedInps = 1;
    private static final double kSlowDesiredExtendSpeedInps = 1;

    // Torque is 2 * 9 * 0.5 = 9
    // Torque on the motor is Torque / ( gearing = 9 ) = 1
    private static final double kVoltsToCounterTorque = 10.5;
    private static final double kSlowVoltsToCounterTorque = (1.1D / 32) * 12;

    public static enum MotorSpeed{retract,extend,slowRetract,slowExtend;};
    private static Dictionary dMotorSpeed = new Hashtable();//given values in constructor


    public static enum EncoderPos{extendLeft,extendRight,retractLeft,retractRight,zero;};
    private static Dictionary dEncoderPos = new Hashtable();//given values in constructor


    public static final int leftMotor = -1;//because someone requested this and there's no point slapping them in an enum
    public static final int bothMotors = 0;
    public static final int rightMotor = 1;


    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight);
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();


    private final Consumer[] setEncoder = new Consumer[]{//faster to array[](inp) than a bunch of if's in a func
      (pos) -> leftEncoder.setPosition((double) pos),
      (pos) -> {leftEncoder.setPosition((double) pos);rightEncoder.setPosition((double) pos);},
      (pos) -> rightEncoder.setPosition((double) pos),
    };

    private final Consumer[] setMotor = new Consumer[]{
      (speed) -> {left.set((double) speed);SmartDashboard.putString("Left Climber State", "Moving");},
      (speed) -> {left.set((double) speed);right.set((double) speed);SmartDashboard.putString("Left Climber State", "Moving");SmartDashboard.putString("Right Climber State", "Moving");},
      (speed) -> {right.set((double) speed);SmartDashboard.putString("Right Climber State", "Moving");},
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

        dEncoderPos.put(EncoderPos.extendRight, 5.315);
        dEncoderPos.put(EncoderPos.retractLeft, -1.151);
        dEncoderPos.put(EncoderPos.extendLeft, -5.317);
        dEncoderPos.put(EncoderPos.retractRight, -1.172);
        dEncoderPos.put(EncoderPos.zero, 0.0);

        dMotorSpeed.put(MotorSpeed.retract, -((kDesiredRetractSpeedInps / kInPerSec) + (kVoltsToCounterTorque / 12))); // ~ -0.06151
        dMotorSpeed.put(MotorSpeed.extend, (kDesiredExtendSpeedInps / kInPerSec)); // ~0.30261
        dMotorSpeed.put(MotorSpeed.slowRetract, -((kSlowDesiredRetractSpeedInps / kInPerSec) + (kSlowVoltsToCounterTorque / 12))); // ~ -0.06151
        dMotorSpeed.put(MotorSpeed.slowExtend, (kSlowDesiredExtendSpeedInps / kInPerSec)); // ~0.30261
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("L Climber Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("R Climber Pos", rightEncoder.getPosition());
    }

    //motor = -1 left or 1 right or 0 both
    public void resetEncodersTo(EncoderPos pos,int motor){
			setEncoder[motor+1].accept(dEncoderPos.get(pos));
    }

    public void moveMotors(MotorSpeed speed, int motor){
      setMotor[motor+1].accept(dMotorSpeed.get(speed));
    }

    public void stopMotors(int motor){
      if (motor<1){
        left.set(0);
        SmartDashboard.putString("Left climber is", "Stopped");
      }
      if (motor>-1){
        right.set(0);
        SmartDashboard.putString("Right Climber State", "Stop");
      }
    }


    public boolean isMotorExtended(int motor){//is the motor(s) extended?
      if (motor<1 && leftEncoder.getPosition() < (double) dEncoderPos.get(EncoderPos.extendLeft)){
        return false;
      }
      if (motor>-1 && rightEncoder.getPosition() < (double) dEncoderPos.get(EncoderPos.extendRight)){
        return false;
      }
      return true;
    }

    public boolean isMotorRetracted(int motor){//is the motor(s) retracted?
      if (motor<1 && leftEncoder.getPosition() > (double) dEncoderPos.get(EncoderPos.retractLeft)){
        return false;
      }
      if (motor>-1 && rightEncoder.getPosition() > (double) dEncoderPos.get(EncoderPos.retractRight)){
        return false;
      }
      return true;
    }

}
