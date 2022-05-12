package org.team199.robot2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.Limelight;
import org.team199.robot2022.subsystems.Drivetrain;

public class AutoIntake extends CommandBase{
    private Drivetrain dt;
    private Limelight lime;
    private double cameraHeight = 1; // TODO fix value
    private double objectHeight = 1; // TODO fix value
    private double cameraAngle = 1; // TODO fix value
    private double tolerance = 0.5; // TODO fix value
    private double adjustment;
    private double[] displacement;
    private boolean foundTarget = false;
    private double wait = 25;
    public AutoIntake(Drivetrain dt, Limelight lime){
        addRequirements(dt);
        this.lime = lime;
        this.dt = dt;
    }
    @Override
    public void execute(){
        adjustment = lime.steeringAssist() * 0.5;
        displacement = lime.determineObjectDist(cameraHeight, objectHeight, cameraAngle);
        for (int i = 0; i < 2; i ++){
            displacement[i] = displacement[i] * 0.4;
        }
        if (lime.getTV() == 0 && !foundTarget){
            displacement[0] = 0;
            displacement[1] = 0;
        }else{
            foundTarget = true;
        }
        if (adjustment <= tolerance){
            adjustment = 0;
        }
        if (foundTarget && lime.getTV() == 0){
            adjustment = 0;
            wait -= 1;
        }
        dt.drive(displacement[0], displacement[1], adjustment);
    }
    @Override
    public void end(boolean interrupted){
        dt.drive(0, 0, 0);
    }
    @Override
    public boolean isFinished(){
        return wait < 0;
    }

}
