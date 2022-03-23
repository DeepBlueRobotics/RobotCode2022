package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class isAtTargetSpeed extends CommandBase{
    private final Shooter shooter;
    public isAtTargetSpeed(Shooter shooter){
        addRequirements(this.shooter = shooter);
    }
    @Override
    public void initialize() {
        // TODO Auto-generated method stub
    }
    @Override
    public void execute() {
        // TODO Auto-generated method stub
    }
    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
    }
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return shooter.isAtTargetSpeed();
    }

}
