package org.team199.robot2022.commands;
import org.team199.robot2022.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ResetAndRetractClimber extends ParallelCommandGroup{
    public ResetAndRetractClimber(Climber climber) {
        super(
            new FunctionalCommand(
                climber::resetEncodersToExtended,
                climber::slowRetractLeft,
                climber::stop,
                climber::isLeftResetRetracted
            ),
            new FunctionalCommand(
                climber::resetEncodersToExtended,
                climber::slowRetractRight,
                climber::stop,
                climber::isRightResetRetracted
            )
        );
        addRequirements(climber);
    }
    
}
