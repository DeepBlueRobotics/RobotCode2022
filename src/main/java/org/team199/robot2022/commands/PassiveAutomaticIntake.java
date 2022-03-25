package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;
import org.team199.robot2022.subsystems.IntakeFeeder.Motor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class PassiveAutomaticIntake extends CommandBase {

    private final IntakeFeeder intakeFeeder;
    private boolean isRegurgitating = false;
    private boolean overrideBottomRoller = true;

    public PassiveAutomaticIntake(IntakeFeeder intakeFeeder) {
        addRequirements(this.intakeFeeder = intakeFeeder);
    }

    @Override
    public void execute() {
        SmartDashboard.putString("IntakeFeeder Default Type", "Automatic");
        intakeFeeder.detectColor();
        // Check for Regurgitation
        if (intakeFeeder.getCargo().size() > 0 && !intakeFeeder.getCargo().peekFirst())
        {
            new Regurgitate(intakeFeeder).schedule();
            
            isRegurgitating = true;
            /*
            intakeFeeder.invertAndRun(Motor.BOTTOM, true, true);
            if (!intakeFeeder.isBallThere(Motor.BOTTOM)){
                intakeFeeder.popFirstBall();
            }
            */
        } /* else {
            isRegurgitating = false;
        }
        */
        
        // Automatically intake balls
        if (!isRegurgitating) {
            // TODO : The ball might not reach the destination fast enough if second ball (Potential error when two balls are intaked instantly)
            switch (intakeFeeder.getNumBalls()) {
                case 0:
                    SmartDashboard.putBoolean("2 Balls in Motor", false);
                    intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, false);
                    intakeFeeder.invertAndRun(Motor.TOP, false, false);
                    // We assume that the case that a ball is stuck after ejection is that it will never happen based on shoot command
                    break;
                case 1:
                    SmartDashboard.putBoolean("2 Balls in Motor", false);
                    intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, true);
                    intakeFeeder.invertAndRun(Motor.TOP, false, false);
                    overrideBottomRoller = true;
                    // The ball is already recorded
                    break;
                case 2:
                    SmartDashboard.putBoolean("2 Balls in Motor", true);
                    if(overrideBottomRoller) { // make sure that the ball gets into the roller
                        intakeFeeder.invertAndRun(Motor.BOTTOM, false, true);
                        if(!intakeFeeder.isBallThere(Motor.BOTTOM)) overrideBottomRoller = false;
                    } else
                        intakeFeeder.invertAndRun(Motor.BOTTOM, false, false);
                    intakeFeeder.invertAndRun(Motor.MIDDLE, false, false);
                    intakeFeeder.invertAndRun(Motor.TOP, false, false);
                    break;
            }

        }
        SmartDashboard.putBoolean("Regurgitating", isRegurgitating);
        isRegurgitating = false;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
