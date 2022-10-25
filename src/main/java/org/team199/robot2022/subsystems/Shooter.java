package org.team199.robot2022.subsystems;

import java.util.stream.Stream;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import org.carlmontrobotics.lib199.LinearActuator;
import org.carlmontrobotics.lib199.LinearInterpolation;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import org.carlmontrobotics.lib199.MotorErrors.TemperatureLimit;
import org.carlmontrobotics.lib199.SparkVelocityPIDController;
import org.team199.robot2022.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private static double kV = 0.129 / 60;
    private static double kS = 0.105;
    private static final double kP = 0.0001;
    private static final double kI = 0.0;
    private static final double kD = 0.005;

    private double kTargetSpeed = 2800;
    private final double speedOffsetMain = 0;
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
    private final LinearInterpolation fenderPos = new LinearInterpolation("fenderPoss.csv");
    private final LinearInterpolation awayFenderPos = new LinearInterpolation("awayFenderPoss.csv");
    private final LinearInterpolation tarmacPos = new LinearInterpolation("tarmacPoss.csv");

    private final CANSparkMax master = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterMaster, TemperatureLimit.NEO);
    private final CANSparkMax slave = MotorControllerFactory.createSparkMax(Constants.DrivePorts.kShooterSlave, TemperatureLimit.NEO);
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
    }

    public void updateFromPSI() {
        double targetPID = chooseFromPosition(fenderRPM, awayFenderRPM, tarmacRPM).calculate(ballPSI);
        pidController.setTargetSpeed(targetPID == 0 ? kTargetSpeed : targetPID);
        setLinearActuatorPos(chooseFromPosition(fenderPos, awayFenderPos, tarmacPos).calculate(ballPSI));
    }

    public void setShotPosition(ShotPosition pos) {
        this.shotPosition = pos;
        updateFromPSI();
    }

    public void toggleLongShot() {
        setLongShot(shotPosition == ShotPosition.FENDER);
    }

    public void setLongShot(boolean isLong) {
        if(isLong) {
            shotPosition = ballPSI <= maxMidShotPSI ? ballPSI >= minMidShotPSI ? ShotPosition.AWAY_FROM_FENDER : ShotPosition.FENDER : ShotPosition.TARMAC;
        } else {
            shotPosition = ShotPosition.FENDER;
        }
        updateFromPSI();
    }

    public void periodic()  {
        if(dutyCycleMode) {
            if(!pidController.isAtTargetSpeed() && !shooterDisabled) {
                master.set(kV * getTargetSpeed() / 12D);
            } else {
                master.set(0);
            }
        }
    }

    public void setMainSpeed(double mainSpeed) {
        pidController.setTargetSpeed(MathUtil.clamp(mainSpeed, 2300, 3200));
    }

    public void setLinearActuatorPos(double value){
        linearActuator.set(MathUtil.clamp(value, linearActuatorMinPos, linearActuatorMaxPos));
    }

    public double getLinearActuatorPos(){
        return linearActuator.get();
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

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Master Current", master::getOutputCurrent, null);
        builder.addDoubleProperty("Slave Current", slave::getOutputCurrent, null);
        builder.addBooleanProperty("isAtTargetSpeed", this::isAtTargetSpeed, null);
        builder.addBooleanProperty("dutyCycleMode", () -> dutyCycleMode, newMode -> dutyCycleMode = newMode);
        builder.addBooleanProperty("shooterDisabled", () -> shooterDisabled, isDisabled -> shooterDisabled = isDisabled);
        builder.addDoubleProperty("Linear Actuator Position", this::getLinearActuatorPos, this::setLinearActuatorPos);
        builder.addDoubleProperty("Ball PSI", () -> ballPSI, psi -> {
            ballPSI = psi;
            updateFromPSI();
        });
        builder.addBooleanProperty("Long Shot", () -> shotPosition != ShotPosition.FENDER, this::setLongShot);
        builder.addStringProperty("Shot Position", () -> shotPosition.toString(), newPos -> shotPosition = Stream.of(ShotPosition.values()).filter(pos -> pos.toString().equals(newPos)).findFirst().orElse(shotPosition));
        addChild("PID Controller", pidController);
    }

    public static enum ShotPosition {
        FENDER, AWAY_FROM_FENDER, TARMAC;
    }

}
