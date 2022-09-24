// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.function.DoubleConsumer;
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

    private boolean keepPosition = true;
    private double holdTolerance = 0.05;

    public static final int leftMotor = -1;//because someone requested this and there's no point slapping them in an enum
    public static final int bothMotors = 0;
    public static final int rightMotor = 1;


    private final CANSparkMax left = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberLeft, TemperatureLimit.NEO);
    private final CANSparkMax right = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kClimberRight, TemperatureLimit.NEO);
    private final RelativeEncoder leftEncoder = left.getEncoder();
    private final RelativeEncoder rightEncoder = right.getEncoder();

    private final DoubleConsumer[] setEncoder = new DoubleConsumer[]{ // faster to array[](inp) than a bunch of if's in a func
      (pos) -> leftEncoder.setPosition(pos),
      (pos) -> {
        leftEncoder.setPosition(pos);
        rightEncoder.setPosition(pos);
      },
      (pos) -> rightEncoder.setPosition(pos),
    };

    private final DoubleConsumer[] setMotor = new DoubleConsumer[]{
      (speed) -> {
        left.set(speed);
        SmartDashboard.putString("Left Climber State", "Moving");
      },
      (speed) -> {
        left.set(speed);
        right.set(speed);
        SmartDashboard.putString("Left Climber State", "Moving");
        SmartDashboard.putString("Right Climber State", "Moving");
      },
      (speed) -> {
        right.set(speed);
        SmartDashboard.putString("Right Climber State", "Moving");
      },
    };

    public Climber() {
        left.setSmartCurrentLimit(80);
        right.setSmartCurrentLimit(80);
        left.setInverted(leftInverted);
        right.setInverted(!leftInverted);

        leftEncoder.setPositionConversionFactor(1 / gearing);
        leftEncoder.setPosition(0);
        rightEncoder.setPositionConversionFactor(1 / gearing);
        rightEncoder.setPosition(0);
        SmartDashboard.putString("Left Climber State", "Stopped");
        SmartDashboard.putString("Right Climber State", "Stopped");
        SmartDashboard.putNumber("kDesiredExtendSpeedInps", kDesiredExtendSpeedInps);
        SmartDashboard.putNumber("kDesiredRetractSpeedInps", kDesiredRetractSpeedInps);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("L Climber Pos", leftEncoder.getPosition());
        SmartDashboard.putNumber("R Climber Pos", rightEncoder.getPosition());

        holdTolerance = SmartDashboard.getNumber("Climber: Tolerance", holdTolerance);
        SmartDashboard.putNumber("Climber: Tolerance", holdTolerance);
        SmartDashboard.putBoolean("Climber: Keep Zeroed", keepPosition);
        keepZeroed();
    }

    public void keepZeroed() {
        if(keepPosition) {
            if(Math.abs(leftEncoder.getPosition()) > holdTolerance) {
                left.set(Math.signum(leftEncoder.getPosition()) > 0 ? MotorSpeed.slowRetract.value : MotorSpeed.slowExtend.value);
            } else {
                left.set(0);
            }
            if(Math.abs(rightEncoder.getPosition()) > holdTolerance) {
                right.set(Math.signum(rightEncoder.getPosition()) > 0 ? MotorSpeed.slowRetract.value : MotorSpeed.slowExtend.value);
            } else {
                right.set(0);
            }
        }
    }

    //motor = -1 left or 1 right or 0 both
    public void resetEncodersTo(EncoderPos posEnum,int motor){
			// setEncoder[motor+1].accept(dEncoderPos.get(pos));
      setEncoder[motor+1].accept(posEnum.value);
    }

    public void moveMotors(MotorSpeed speedEnum, int motor){
      setMotor[motor+1].accept(speedEnum.value);
    }

    public void stopMotors(int motor){
      if (motor<1){
        left.set(0);
        SmartDashboard.putString("Left Climber State", "Stopped");
      }
      if (motor>-1){
        right.set(0);
        SmartDashboard.putString("Right Climber State", "Stopped");
      }
    }

    public boolean isMotorExtended(int motor){//is the motor(s) extended?
      if (motor<1 && leftEncoder.getPosition() < EncoderPos.extendLeft.value){
        return false;
      }
      if (motor>-1 && rightEncoder.getPosition() < EncoderPos.extendRight.value){
        return false;
      }
      return true;
    }

    public boolean isMotorRetracted(int motor){//is the motor(s) retracted?
      if (motor<1 && leftEncoder.getPosition() > EncoderPos.retractLeft.value){
        return false;
      }
      if (motor>-1 && rightEncoder.getPosition() > EncoderPos.retractRight.value){
        return false;
      }
      return true;
    }

    public static enum MotorSpeed{
      retract(-((kDesiredRetractSpeedInps / kInPerSec) + (kVoltsToCounterTorque / 12))),// ~ -0.06151
      extend(kDesiredExtendSpeedInps / kInPerSec),// ~0.30261
      slowRetract(-((kSlowDesiredRetractSpeedInps / kInPerSec) + (kSlowVoltsToCounterTorque / 12))),// ~ -0.06151
      slowExtend(kSlowDesiredExtendSpeedInps / kInPerSec);// ~0.30261

      public final double value;

      private MotorSpeed(double value) {
          this.value = value;
      }
    };


    public static enum EncoderPos{
      extendLeft(-5.317),
      extendRight(5.315),
      retractLeft(-1.151),
      retractRight(-1.172),
      zero(0.0);

      public final double value;

      private EncoderPos(double value) {
          this.value = value;
      }
    };

}
