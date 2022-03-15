package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ResetAndExtendClimber extends ParallelCommandGroup{
    public ResetAndExtendClimber(Climber climber) {
        super(
            new FunctionalCommand(
                climber::resetEncodersToRetracted,
                climber::slowExtendLeft,
                climber::stopLeft,
                climber::isLeftResetExtended
            ),
            new FunctionalCommand(
                climber::resetEncodersToRetracted,
                climber::slowExtendRight,
                climber::stopRight,
                climber::isRightResetExtended
            )
        );
        addRequirements(climber);
    }
    
}
