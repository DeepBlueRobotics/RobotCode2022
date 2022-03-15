package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;

import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.SparkVelocityPIDController;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;

    // Target speeds
    private double kUpperHubTarget = 2400;
    private double kLowerHubTarget = 1000;
    private double kSoftShootTarget = 700;
    //private double kTargetSpeed = 2500;
    private final double speedOffsetMain = 100;
    public double kTargetSpeed = kUpperHubTarget;

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterSlave);
    private final SparkVelocityPIDController pidController = new SparkVelocityPIDController("Shooter", master, kP, kI, kD, kS, kV, kTargetSpeed, speedOffsetMain);

    public Shooter() {
        master.setSmartCurrentLimit(40);
        slave.setSmartCurrentLimit(40);
        SmartDashboard.putNumber("kUpperHubTarget", kUpperHubTarget);
        SmartDashboard.putNumber("kLowerHubTarget", kLowerHubTarget);
        SmartDashboard.putNumber("kSoftShootTarget", kSoftShootTarget);

        slave.follow(master, true);
        master.setInverted(true);
    }

    public void periodic()  {
        pidController.periodic();
        SmartDashboard.getNumber("kUpperHubTarget", kUpperHubTarget);
        SmartDashboard.getNumber("kLowerHubTarget", kLowerHubTarget);
        SmartDashboard.getNumber("kSoftShootTarget", kSoftShootTarget);
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
        return pidController.isAtTargetSpeed();
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
