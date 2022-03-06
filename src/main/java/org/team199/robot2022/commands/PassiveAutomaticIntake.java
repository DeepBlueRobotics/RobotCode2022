package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PassiveAutomaticIntake extends CommandBase {

    private final IntakeFeeder intakeFeeder;
    private boolean isRegurgitating = false;

    public PassiveAutomaticIntake(IntakeFeeder intakeFeeder) {
        addRequirements(this.intakeFeeder = intakeFeeder);
    }

    @Override
    public void execute() {
        intakeFeeder.detectColor();
        // Check for Regurgitation
        if (intakeFeeder.getCargo().size() > 0 && !intakeFeeder.getCargo().peek())
        {
            isRegurgitating = true;
            intakeFeeder.invertAndRun(Motor.BOTTOM, true, true);
        }
        if (isRegurgitating && !intakeFeeder.isBallThere(Motor.BOTTOM)){
            intakeFeeder.popBall();
        }
        // Automatically intake balls
        if (!isRegurgitating) {
            // TODO : The ball might not reach the destination fast enough if second ball (Potential error when two balls are intaked instantly)
            switch (intakeFeeder.getNumBalls()) {
                case 0:
                    intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, true);
                    intakeFeeder.invertAndRun(Motor.TOP, false, true);
                    // We assume that the case that a ball is stuck after ejection is that it will never happen based on shoot command
                    break;
                case 1:
                    intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, true);
                    intakeFeeder.invertAndRun(Motor.TOP, false, false);
                    // The ball is already recorded
                    break;
                case 2:
                    intakeFeeder.invertAndRun(Motor.BOTTOM, false, false);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, false);
                    intakeFeeder.invertAndRun(Motor.TOP, false, false);
                    break;
            }

        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
