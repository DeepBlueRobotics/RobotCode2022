package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PassiveManualIntake extends CommandBase {

    private final IntakeFeeder intakeFeeder;
    private boolean isSpinningUp = true;
    private int lastFeed = 0;

    public PassiveManualIntake(IntakeFeeder intakeFeeder) {
        addRequirements(this.intakeFeeder = intakeFeeder);
    }

    @Override
    public void execute() {
        // feed = (int) SmartDashboard.getNumber("Size", feed);
        SmartDashboard.putString("Detected Color", "Disconnected");
        SmartDashboard.putString("IntakeFeeder Default Type", "Manual");
        // Reset the balls in the cargo as color sensor no longer works and we cannot
        // Manually intake balls
        if(lastFeed != intakeFeeder.getNumBalls()) isSpinningUp = true;
        lastFeed = intakeFeeder.getNumBalls();
        switch (intakeFeeder.getNumBalls()) {
            case 0:
                intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                intakeFeeder.invertAndRun(Motor.MIDDLE, false, true);
                intakeFeeder.invertAndRun(Motor.TOP, false, true);

                if(intakeFeeder.isBallThere(Motor.TOP)) {
                    if(!isSpinningUp) {
                        intakeFeeder.manualAdd();
                    }
                } else {
                    isSpinningUp = false;
                }
                break;
            case 1:
                intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                intakeFeeder.invertAndRun(Motor.MIDDLE, false, true);
                intakeFeeder.invertAndRun(Motor.TOP, false, false);

                if(intakeFeeder.isBallThere(Motor.MIDDLE)) {
                    if(!isSpinningUp) {
                        intakeFeeder.manualAdd();
                    }
                } else {
                    isSpinningUp = false;
                }
                break;
            case 2:
                intakeFeeder.invertAndRun(Motor.BOTTOM, false, false);
                intakeFeeder.invertAndRun(Motor.MIDDLE, false, false);
                intakeFeeder.invertAndRun(Motor.TOP, false, false);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
