package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.LinearActuator;
import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.SparkVelocityPIDController;
import frc.robot.lib.logging.Log;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;
    private final double speedOffsetMain = 0;
    // Target speeds
    private double kUpperHubTarget = 2800;
    private double kLowerHubTarget = 1000;
    private double kSoftShootTarget = 700;
    //private double kTargetSpeed = 2500;
    public double kTargetSpeed = kUpperHubTarget;

    private double linearActuatorPos = 0;
    private final LinearActuator linearActuator = new LinearActuator(0, 0, 50);

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterSlave);
    private final SparkVelocityPIDController pidController = new SparkVelocityPIDController("Shooter", master, kP, kI, kD, kS, kV, kTargetSpeed, speedOffsetMain) {
        @Override
        public void setTargetSpeed(double targetSpeed) {
            if(!isDutyCycleMode()) super.setTargetSpeed(targetSpeed);
        }
    };

    private boolean dutyCycleMode = false;
    private boolean shooterDisabled = false;

    public Shooter() {
        master.setSmartCurrentLimit(40);
        slave.setSmartCurrentLimit(40);
        SmartDashboard.putNumber("kUpperHubTarget", kUpperHubTarget);
        SmartDashboard.putNumber("kLowerHubTarget", kLowerHubTarget);
        SmartDashboard.putNumber("kSoftShootTarget", kSoftShootTarget);

        slave.follow(master, true);
        master.setInverted(true);

        Log.registerDoubleVar("Shooter RPM", () -> pidController.getEncoder().getVelocity());
        Log.registerDoubleVar("Shooter Current Master", () -> master.getOutputCurrent());
        Log.registerDoubleVar("Shooter Current Slave", () -> slave.getOutputCurrent());
    }

    public void periodic()  {
        pidController.periodic();
        SmartDashboard.putNumber("Actual Speed: ", master.getEncoder().getVelocity());
        SmartDashboard.putBoolean("isAtTargetSpeed", isAtTargetSpeed());
        SmartDashboard.putNumber("kTargetSpeed", kTargetSpeed);
        SmartDashboard.putString("Shooter: Mode", dutyCycleMode ? "Duty Cycle" : "PID");
        SmartDashboard.putBoolean("Shooter Disabled", shooterDisabled);
        linearActuatorPos = SmartDashboard.getNumber("Linear Actuator Position", linearActuatorPos);
        SmartDashboard.putNumber("Linear Actuator Position", linearActuatorPos);
        linearActuator.set(linearActuatorPos);
        kTargetSpeed = SmartDashboard.getNumber("kTargetSpeed", kTargetSpeed);
        SmartDashboard.getNumber("kUpperHubTarget", kUpperHubTarget);
        SmartDashboard.getNumber("kLowerHubTarget", kLowerHubTarget);
        SmartDashboard.getNumber("kSoftShootTarget", kSoftShootTarget);
        if(dutyCycleMode) {
            if(!pidController.isAtTargetSpeed() && !shooterDisabled) {
                master.set(kV * getTargetSpeed() / 12D);
            } else {
                master.set(0);
            }
        }
    }

    public void setMainSpeed(ShootMode mode) {
        switch(mode)
        {
            case UPPER:
                kTargetSpeed = kUpperHubTarget;
                break;
            case LOWER:
                kTargetSpeed = kLowerHubTarget;
                break;
            case SOFT:
                kTargetSpeed = kSoftShootTarget;
                break;
        }
    }

    public double getTargetSpeed() {
        return kTargetSpeed;
    }

    public boolean isAtTargetSpeed() {
        //return pidController.isAtTargetSpeed();
        return master.getEncoder().getVelocity() > kTargetSpeed - speedOffsetMain;
    }

    public void toggleDutyCycleMode() {
        dutyCycleMode = !dutyCycleMode;
    }

    public void disableShooter() {
        shooterDisabled = true;
    }

    public void enableShooter() {
        shooterDisabled = false;
    }

    public boolean isDutyCycleMode() {
        return dutyCycleMode;
    }
    
    //if motor velocity is slower than usual, returns a boolean
    public boolean isBallThere() {
        return !isAtTargetSpeed();
    }

    public enum ShootMode {
        UPPER,
        LOWER,
        SOFT
    }
}
