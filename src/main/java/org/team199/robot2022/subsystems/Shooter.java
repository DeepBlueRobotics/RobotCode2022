package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

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

    private double kTargetSpeed = 2800;
    private double linearActuatorPos = 0;
    private final double speedOffsetMain = 0;
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

        slave.follow(master, true);
        master.setInverted(true);

        master.setIdleMode(IdleMode.kCoast);
        slave.setIdleMode(IdleMode.kCoast);

        Log.registerDoubleVar("Shooter RPM", () -> pidController.getEncoder().getVelocity());
        Log.registerDoubleVar("Shooter Current Master", () -> master.getOutputCurrent());
        Log.registerDoubleVar("Shooter Current Slave", () -> slave.getOutputCurrent());
    }

    public void periodic()  {
        pidController.periodic();
        SmartDashboard.putNumber("Actual Speed: ", master.getEncoder().getVelocity());
        SmartDashboard.putBoolean("isAtTargetSpeed", isAtTargetSpeed());
        SmartDashboard.putString("Shooter: Mode", dutyCycleMode ? "Duty Cycle" : "PID");
        SmartDashboard.putBoolean("Shooter Disabled", shooterDisabled);
        SmartDashboard.putNumber("Linear Actuator Position", linearActuatorPos);
        linearActuator.set(SmartDashboard.getNumber("LinearActuator Position", linearActuatorPos));
        if(dutyCycleMode) {
            if(!pidController.isAtTargetSpeed() && !shooterDisabled) {
                master.set(kV * getTargetSpeed() / 12D);
            } else {
                master.set(0);
            }
        }
    }

    public void setMainSpeed(double mainSpeed) {
        pidController.setTargetSpeed(mainSpeed);
    }

    public double getTargetSpeed() {
        return pidController.getTargetSpeed();
    }

    public boolean isAtTargetSpeed() {
        return dutyCycleMode ? pidController.getEncoder().getVelocity() + 100 >= pidController.getTargetSpeed() : pidController.isAtTargetSpeed();
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

}
