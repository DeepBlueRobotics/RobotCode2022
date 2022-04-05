package org.team199.robot2022.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.team199.robot2022.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.SpeedController;
//import java.lang.AutoCloseable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.SparkVelocityPIDController;
import frc.robot.lib.LinearActuator;
import frc.robot.lib.LinearInterpolation;
import frc.robot.lib.logging.Log;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;

    private double kTargetSpeed = 2800;
    private final double speedOffsetMain = 0;
    private double linearActuatorPos = 0;
    private final LinearActuator linearActuator = new LinearActuator(0, 0, 50);
    private final double linearActuatorMaxPos = 1;
    private final double linearActuatorMinPos = 0;

    private ShotPosition shotPosition = ShotPosition.FENDER;
    private double ballPSI = 3.5;
    private final double minMidShotPSI = 2.25;
    private final double maxMidShotPSI = 2.675;
    private final LinearInterpolation fenderRPM = new LinearInterpolation("fenderRPMs.csv");
    private final LinearInterpolation awayFenderRPM = new LinearInterpolation("awayFenderRPMs.csv");
    private final LinearInterpolation tarmacRPM = new LinearInterpolation("tarmacRPMs.csv");
    private final LinearInterpolation fenderPos = new LinearInterpolation("fenderPos.csv");
    private final LinearInterpolation awayFenderPos = new LinearInterpolation("awayFenderPoss.csv");
    private final LinearInterpolation tarmacPos = new LinearInterpolation("tarmacPoss.csv");

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
        
        SmartDashboard.putNumber("BallPSI", ballPSI);
        SmartDashboard.putBoolean("Long Shot", shotPosition != ShotPosition.FENDER);

        Log.registerDoubleVar("Shooter RPM", () -> pidController.getEncoder().getVelocity());
        Log.registerDoubleVar("Shooter Current Master", () -> master.getOutputCurrent());
        Log.registerDoubleVar("Shooter Current Slave", () -> slave.getOutputCurrent());
    }

    public void updateFromPSI() {
        double targetPID = chooseFromPosition(fenderRPM, awayFenderRPM, tarmacRPM).calculate(ballPSI);
        pidController.setTargetSpeed(targetPID == 0 ? kTargetSpeed : targetPID);
        SmartDashboard.putNumber("Linear Actuator Position", chooseFromPosition(fenderPos, awayFenderPos, tarmacPos).calculate(ballPSI));
    }

    public void setShotPosition(ShotPosition pos) {
        this.shotPosition = pos;
        updateFromPSI();
    }

    public void toggleLongShot() {
        if(shotPosition == ShotPosition.FENDER) {
            shotPosition = ballPSI <= maxMidShotPSI ? ballPSI >= minMidShotPSI ? ShotPosition.AWAY_FROM_FENDER : ShotPosition.FENDER : ShotPosition.TARMAC;
        } else {
            shotPosition = ShotPosition.FENDER;
        }
    }

    public void periodic()  {
        pidController.periodic();
        SmartDashboard.putBoolean("isAtTargetSpeed", isAtTargetSpeed());
        SmartDashboard.putString("Shooter: Mode", dutyCycleMode ? "Duty Cycle" : "PID");
        SmartDashboard.putBoolean("Shooter Disabled", shooterDisabled);
        linearActuatorPos = SmartDashboard.getNumber("Linear Actuator Position", linearActuatorPos);
        SmartDashboard.putNumber("Linear Actuator Position", linearActuatorPos);
        linearActuator.set(linearActuatorPos);
        ballPSI = SmartDashboard.getNumber("Ball PSI", ballPSI);

        SmartDashboard.putString("Shot Position", shotPosition.toString());

        if(dutyCycleMode) {
            if(!pidController.isAtTargetSpeed() && !shooterDisabled) {
                master.set(kV * getTargetSpeed() / 12D);
            } else {
                master.set(0);
            }
        }
    }

    public void setMainSpeed(double mainSpeed) {
        if (mainSpeed > 3200){
            pidController.setTargetSpeed(3200);
        }else if (mainSpeed < 2300){
            pidController.setTargetSpeed(2300);
        }else{
            pidController.setTargetSpeed(mainSpeed);
        }
    }

    public void setLinearActuatorPos(double value){
        if (value > linearActuatorMaxPos){
            linearActuatorPos = linearActuatorMaxPos;
        }else if (value < linearActuatorMinPos){
            linearActuatorPos = linearActuatorMinPos;
        }else{
            linearActuatorPos = value;
        }
        SmartDashboard.putNumber("Linear Actuator Position", linearActuatorPos);
    }
    public double getLinearActuatorPos(){
        return linearActuatorPos;
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

    public <T> T chooseFromPosition(T fender, T awayFender, T tarmac) {
        switch(shotPosition) {
            case FENDER:
                return fender;
            case AWAY_FROM_FENDER:
                return awayFender;
            case TARMAC:
                return tarmac;
            default:
                System.err.println("Unknown case: " + shotPosition + "! Assuming FENDER :/");
                return fender;
        }
    }

    public static enum ShotPosition {
        FENDER, AWAY_FROM_FENDER, TARMAC;
    }

}
