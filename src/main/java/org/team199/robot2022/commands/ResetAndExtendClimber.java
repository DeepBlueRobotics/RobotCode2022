package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ResetAndExtendClimber extends CommandBase {
    private final Climber climber;
    
    public ResetAndExtendClimber(Climber climber) {
        addRequirements(this.climber = climber);
    }

    @Override
    public void initialize() {
        climber.resetEncodersTo(climber.EncoderPos.retractLeft,-1);
        climber.resetEncodersTo(climber.EncoderPos.retractRight,1);
        climber.moveMotors(climber.MotorSpeed.kSlowExtendSpeed,0);
    }

    @Override
    public boolean isFinished(){
        return (isMotorExtended(-1,true) && isMotorExtended(1,true));
    }

    @Override
    public void end(){
        climber.stopMotors(0);
    }

}
