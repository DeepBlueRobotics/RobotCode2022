package org.team199.robot2022.commands;

import org.team199.robot2022.subsystems.IntakeFeeder;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PassiveAutomaticIntake extends CommandBase {

    private final IntakeFeeder intakeFeeder;

    public PassiveAutomaticIntake(IntakeFeeder intakeFeeder) {
        addRequirements(this.intakeFeeder = intakeFeeder);
    }

    @Override
    public void execute() {
        // feed = cargo.size();
        // if (cargo.size() < 2) {
        //     bottom.set(botSpeed);
        // }

        // if (cargo.size() > 0 && cargo.peekFirst() == false) {
        //     middle.set(0);
        //     // Regurgitate via intake
        //     if (!isJammed(bottom) && isBallThere(bottom)) {
        //         bottom.setInverted(botInverted);
        //         bottom.set(botSpeed);
        //     } else {
        //         // TODO : cargo.removeFirst() may remove the red ball but the color sensor will
        //         // not detect that the ball is not
        //         // regurgitated and will not add the ball
        //         cargo.removeFirst();
        //         bottom.setInverted(!botInverted);
        //         bottom.set(botSpeed);
        //     }
        // }
        // if (detectColor()) {
        //     if (cargo.size() == 1 && cargo.peekFirst()) {
        //         // while ball is still in color sensor range move the ball out to prevent jam
        //         // TODO : The ball might not reach the destination fast enough if second ball
        //         // gets in
        //         if ((!isBallThere(middle) && isBallThere(top)) || (middle.get() == 0 && top.get() == 0 && !initRunMid)) // TODO
        //                                                                                                                 // :
        //                                                                                                                 // Fix
        //                                                                                                                 // this,
        //                                                                                                                 // logic
        //                                                                                                                 // is
        //                                                                                                                 // wrong
        //         {
        //             middle.set(0);
        //             top.set(0);
        //         } else {
        //             initRunMid = false;
        //             if (!isJammed(middle)) {
        //                 middle.set(midSpeed);
        //                 top.set(topSpeed);
        //             } else if (isBallThere(middle)) {
        //                 unJam(middle, midSpeed, midInverted);
        //             }
        //         }
        //     }
        //     // If this ball is the second ball in the feeder
        //     else {
        //         // This is to prevent any more balls getting in
        //         top.set(0);
        //         middle.set(0);
        //         bottom.set(0);
        //         bottom.setInverted(!botInverted);
        //     }
        // }
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
