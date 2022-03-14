// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team199.robot2022;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.lib.SwerveConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double g = 9.81; //meters per second squared

    public static final class DriveConstants {

        public static final double wheelBase = Units.inchesToMeters(20.0);
        public static final double trackWidth = Units.inchesToMeters(21.0);
        // "swerveRadius" is the distance from the center of the robot to one of the modules
        public static final double swerveRadius = Math.sqrt(Math.pow(wheelBase / 2, 2) + Math.pow(trackWidth / 2, 2));
        // The gearing reduction from the drive motor controller to the wheels
        // Gearing for the Swerve Modules is 6.75 : 1
        public static final double driveGearing = 6.75;
        public static final double driveToRungDist = 0;
        //TODO : this is not 0, find and set
        public static final double driveToRungSpeed = 0;
        //TODO : this is not 0, find and set

        public static final double driveModifier = 1;
        public static final double wheelDiameterMeters = Units.inchesToMeters(4.0) * 7.36/7.65 /* empirical correction */;
        public static final double mu = 0.5; /* 70/83.2;  */

        public static final double NEOFreeSpeed = 5676 * (2 * Math.PI) / 60;    // radians/s
        // Angular speed to translational speed --> v = omega * r / gearing
        public static final double maxSpeed = NEOFreeSpeed * (wheelDiameterMeters / 2.0) / driveGearing * 1.0;
        public static final double maxForward = maxSpeed;
        public static final double maxStrafe = maxSpeed;
        // maxRCW is the angular velocity of the robot.
        // Calculated by looking at one of the motors and treating it as a point mass moving around in a circle.
        // Tangential speed of this point mass is maxSpeed and the radius of the circle is sqrt((wheelBase/2)^2 + (trackWidth/2)^2)
        // Angular velocity = Tangential speed / radius
        public static final double maxRCW = maxSpeed / swerveRadius;

        public static final boolean[] reversed = {false, false, false, false};
        // Determine correct turnZero constants (FL, FR, BL, BR)
        public static final double[] turnZero = {-72.861, 47.549, -178.505, 66.885};

        // kP, kI, and kD constants for turn motor controllers in the order of front-left, front-right, back-left, back-right.
        // Determine correct turn PID constants
        public static final double[] turnkP = {0.00374, 0.00374, 0.00374, 0.00374};
        //public static final double[] turnkP = {0.005, 0.005, 0.005, 0.005};
        public static final double[] turnkI = {0, 0, 0, 0};
        public static final double[] turnkD = {0, 0, 0, 0};
        public static final double[] turnkS = {0.2, 0.2, 0.2, 0.2};
        // V = kS + kV * v + kA * a
        // 12 = 0.2 + 0.00463 * v
        // v = (12 - 0.2) / 0.00463 = 2548.596 degrees/s
        public static final double[] turnkV = {0.00463, 0.00463, 0.00463, 0.00463};
        public static final double[] turnkA = {0.000115, 0.000115, 0.000115, 0.000115};
        //public static final double[] turnkD = {0.0001, 0.0001, 0.0001, 0.0001};

        // kP is an average of the forward and backward kP values
        // Forward: 1.72, 1.71, 1.92, 1.94
        // Backward: 1.92, 1.92, 2.11, 1.89
        public static final double[] drivekP = {1.82, 1.815, 2.015, 1.915};
        // public static final double[] drivekP = {0, 0, 0, 0};
        public static final double[] drivekI = {0, 0, 0, 0};
        // public static final double[] drivekD = {.1,.1,.1,.1};
        public static final double[] drivekD = {0,0,0,0};
        public static final boolean[] driveInversion = {true, true, true, true};

        public static final double[] kForwardVolts = {0.129, 0.108, 0.14, 0.125};
        public static final double[] kBackwardVolts = {0.115, 0.169, 0.13, 0.148};

        //public static final double[] kForwardVolts = {0, 0, 0, 0};
        //public static final double[] kBackwardVolts = {0, 0, 0, 0};
        public static final double[] kForwardVels = {2.910/1.1, 2.970/1.1, 2.890/1.1, 2.930/1.1};
        public static final double[] kBackwardVels = {2.890/1.1, 2.800/1.1, 2.850/1.1, 2.820/1.1};
        public static final double[] kForwardAccels = {0.145, 0.149, 0.192, 0.198};
        public static final double[] kBackwardAccels = {0.192, 0.187, 0.264, 0.176};
        //public static final double[] kForwardAccels = {0, 0, 0, 0};
        //public static final double[] kBackwardAccels = {0, 0, 0, 0};

        public static final double autoMaxSpeedMps = 0.35 * 4.4;  // Meters / second
        public static final double autoMaxAccelMps2 = mu * g * 0.6;  // Meters / seconds^2
        public static final double autoMaxVolt = 10.0;   // For Drivetrain voltage constraint in RobotPath.java
        // The maximum acceleration the robot can achieve is equal to the coefficient of static friction times the gravitational acceleration
        // a = mu * 9.8 m/s^2
        public static final double autoCentripetalAccel = mu * g * 0.3;

        // PID values are listed in the order kP, kI, and kD
        public static final double[] xPIDController = {4, 0.0, 0.0};
        public static final double[] yPIDController = {4, 0.0, 0.0};
        public static final double[] thetaPIDController = {4, 0.0, 0.0};

        public static final SwerveConfig swerveConfig = new SwerveConfig(wheelDiameterMeters, driveGearing, mu, autoCentripetalAccel, kForwardVolts, kForwardVels, kForwardAccels, kBackwardVolts, kBackwardVels, kBackwardAccels, drivekP, drivekI, drivekD, turnkP, turnkI, turnkD, turnkS, turnkV, turnkA, turnZero, driveInversion, reversed);

    }

    public static final class DrivePorts {

        public static final int driveFrontLeft = 10;
        public static final int driveFrontRight = 2;
        public static final int driveBackLeft = 11;
        public static final int driveBackRight = 17;

        public static final int turnFrontLeft = 9;
        public static final int turnFrontRight = 1;
        public static final int turnBackLeft = 12;
        public static final int turnBackRight = 18;

        public static final int canCoderPortFL = 1;
        public static final int canCoderPortFR = 2;
        public static final int canCoderPortBL = 3;
        public static final int canCoderPortBR = 4;

        public static final int kShooterMaster = 19;
        public static final int kShooterSlave = 20;
        public static final int kShooterTop = 99;

        public static final int kIntakeTop = 3;
        public static final int kIntakeMiddle = 4;
        public static final int kIntakeBottom = 5;

    }

    public static final class OI {

        public static enum ControlType {JOYSTICKS, GAMEPAD};
        public static enum StickType {LEFT, RIGHT};
        public static enum StickDirection {X, Y};

        public static ControlType CONTROL_TYPE = ControlType.JOYSTICKS;
        public static final double JOY_THRESH = 0.01;
        public static final class LeftJoy {
            public static final int port = 0;
            public static final int manualAddPort = 2; // TODO: Set correct port for addPort, subtractPort, regurgitatePort, and overridePort
            public static final int manualSubtractPort = 3;
            public static final int overridePort = 5;
        }

        public static final class RightJoy {
            public static final int port = 1;
            public static final int shootPort = 2;
            public static final int shootSoftOnePort = 4;

            public static final int runIntakeForwardPort = 5; // TODO: set correct port
            public static final int runIntakeBackwardPort = 3; // TODO: set correct port
            public static final int regurgitatePort = 6;
            public static final int shootLowerHubPort = 7;


        }

        public static final class Controller {
            public static final int port = 2;

            public static Joystick controller = new Joystick(port);

            public static int X;
            public static int A;
            public static int B;
            public static int Y;
            public static int LB;
            public static int RB;
            public static int LT;
            public static int RT;
            public static int BACK;
            public static int START;

            //TODO: mode button setting to teletop init
            static {
                if (controller.getName().equals("Logitech Dual Action")) {
                    // Buttons and triggers
                    X = 1;
                    A = 2;
                    B = 3;
                    Y = 4;
                    LB = 5;
                    RB = 6;
                    LT = 7;
                    RT = 8;
                    BACK = 9;
                    START = 10;
                } else {
                    // Buttons and triggers for xbox controller
                    X = 3;
                    A = 1;
                    B = 2;
                    Y = 4;
                    LB = 5;
                    RB = 6;
                    LT = 7;
                    RT = 8;
                    BACK = 9;
                    START = 10;
                }
            }

            public static final int midClimberDeploy = LT;
            public static final int midClimberRetract = LB;
            public static final int highClimberDeploy = RT;
            public static final int highClimberRetract = RB;

            public static final int runIntakeForwardPort = A; // TODO: set correct port
            public static final int runIntakeBackwardPort = X; // TODO: set correct port
            public static final int regurgitatePort = B;
            public static final int dumbModeToggle = Y;
            public static final int toggleIntakePort = START;
        }
    }

}
