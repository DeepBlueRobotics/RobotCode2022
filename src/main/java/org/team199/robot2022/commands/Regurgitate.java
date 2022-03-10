package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Regurgitate extends CommandBase {

    private final IntakeFeeder intakeFeeder;
    private boolean ballDetected;

    public Regurgitate(IntakeFeeder intakeFeeder) {
        addRequirements(this.intakeFeeder = intakeFeeder);
    }

    @Override
    public void initialize() {
        ballDetected = false;
    }

    @Override
    public void execute() {
        intakeFeeder.runBackward();
        if(intakeFeeder.isBallThere(IntakeFeeder.Motor.BOTTOM)) {
            ballDetected = true;
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted) return;
        intakeFeeder.stopRunningFeeder();
        intakeFeeder.popFirstBall();
    }

    @Override
    public boolean isFinished() {
        return intakeFeeder.useAutonomousControl() ? ballDetected && !intakeFeeder.isBallThere(IntakeFeeder.Motor.BOTTOM) : true;
    }

}
