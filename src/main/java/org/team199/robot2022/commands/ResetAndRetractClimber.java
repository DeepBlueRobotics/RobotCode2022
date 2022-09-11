package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ResetAndRetractClimber extends ParallelCommandGroup{
    private final Climber climber;

    public ResetAndRetractClimber(Climber climber) {
        addRequirements(this.climber = climber);
    }

    @Override
    public void initialize() {
        climber.resetEncodersTo(climber.EncoderPos.extendtLeft,-1);
        climber.resetEncodersTo(climber.EncoderPos.extendRight,1);
        climber.moveMotors(climber.MotorSpeed.kSlowRetractSpeed,0);
    }

    @Override
    public boolean isFinished(){
        return (isMotorRetracted(-1,true) && isMotorRetracted(1,true));
    }

    @Override
    public void end(){
        climber.stopMotors(0);
    }

}
