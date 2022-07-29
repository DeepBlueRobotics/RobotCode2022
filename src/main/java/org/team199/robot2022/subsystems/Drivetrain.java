package org.team199.robot2022.subsystems;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import org.team199.robot2022.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.MotorControllerFactory;
import frc.robot.lib.SwerveModule;
import frc.robot.lib.MotorErrors.TemperatureLimit;
import frc.robot.lib.path.SwerveDriveInterface;

public class Drivetrain extends SubsystemBase implements SwerveDriveInterface {
    // compassOffset is magnetic north relative to the current heading
    private final double compassOffset;
    private final AHRS gyro = new AHRS(SerialPort.Port.kUSB); // Also try kUSB and kUSB2

    private SwerveDriveKinematics kinematics = null;
    private SwerveDriveOdometry odometry = null;
    private SwerveModule modules[];
    private static final boolean isGyroReversed = true;
    private static boolean fieldOriented = true;
    private double initTimestamp = 0;
    private final float initPitch;
    private final float initRoll;

    public Drivetrain() {
        gyro.calibrate();
        initTimestamp = Timer.getFPGATimestamp();
        double currentTimestamp = initTimestamp;
        while (gyro.isCalibrating() && currentTimestamp - initTimestamp < 10) {
            currentTimestamp = Timer.getFPGATimestamp();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
                break;
            }
            System.out.println("Calibrating the gyro...");
        }
        gyro.reset();
        System.out.println("NavX-MXP firmware version: " + gyro.getFirmwareVersion());
        SmartDashboard.putBoolean("Magnetic Field Disturbance", gyro.isMagneticDisturbance());
        System.out.println("Magnetometer is calibrated: " + gyro.isMagnetometerCalibrated());
        compassOffset = gyro.getCompassHeading();
        // Define the corners of the robot relative to the center of the robot using
        // Translation2d objects.
        // Positive x-values represent moving toward the front of the robot whereas
        // positive y-values represent moving toward the left of the robot.
        Translation2d locationFL = new Translation2d(Constants.DriveConstants.wheelBase / 2,
                Constants.DriveConstants.trackWidth / 2);
        Translation2d locationFR = new Translation2d(Constants.DriveConstants.wheelBase / 2,
                -Constants.DriveConstants.trackWidth / 2);
        Translation2d locationBL = new Translation2d(-Constants.DriveConstants.wheelBase / 2,
                Constants.DriveConstants.trackWidth / 2);
        Translation2d locationBR = new Translation2d(-Constants.DriveConstants.wheelBase / 2,
                -Constants.DriveConstants.trackWidth / 2);

        kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(Units.degreesToRadians(getHeading())));
        initPitch = 0;
        initRoll = 0;
        Supplier<Float> pitchSupplier = () -> initPitch;
        Supplier<Float> rollSupplier = () -> initRoll;
        SwerveModule moduleFL = new SwerveModule(Constants.DriveConstants.swerveConfig, SwerveModule.ModuleType.FL,
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveFrontLeft, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnFrontLeft, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(Constants.DrivePorts.canCoderPortFL), Constants.DriveConstants.driveModifier,
                Constants.DriveConstants.maxSpeed, 0, pitchSupplier, rollSupplier);
        // Forward-Right
        SwerveModule moduleFR = new SwerveModule(Constants.DriveConstants.swerveConfig, SwerveModule.ModuleType.FR,
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveFrontRight, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnFrontRight, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(Constants.DrivePorts.canCoderPortFR), Constants.DriveConstants.driveModifier,
                Constants.DriveConstants.maxSpeed, 1, pitchSupplier, rollSupplier);
        // Backward-Left
        SwerveModule moduleBL = new SwerveModule(Constants.DriveConstants.swerveConfig, SwerveModule.ModuleType.BL,
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveBackLeft, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnBackLeft, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(Constants.DrivePorts.canCoderPortBL), Constants.DriveConstants.driveModifier,
                Constants.DriveConstants.maxSpeed, 2, pitchSupplier, rollSupplier);
        // Backward-Right
        SwerveModule moduleBR = new SwerveModule(Constants.DriveConstants.swerveConfig, SwerveModule.ModuleType.BR,
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.driveBackRight, TemperatureLimit.NEO),
                MotorControllerFactory.createSparkMax(Constants.DrivePorts.turnBackRight, TemperatureLimit.NEO),
                MotorControllerFactory.createCANCoder(Constants.DrivePorts.canCoderPortBR), Constants.DriveConstants.driveModifier,
                Constants.DriveConstants.maxSpeed, 3, pitchSupplier, rollSupplier);
        modules = new SwerveModule[] { moduleFL, moduleFR, moduleBL, moduleBR };

        SmartDashboard.putBoolean("Teleop Face Direction of Travel", false);
        SmartDashboard.putBoolean("Field Oriented", true);
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++) {
            modules[i].periodic();
            // Uncommenting the following line will contribute to loop overrun errors
            // modules[i].updateSmartDashboard();
        }

        // Update the odometry with current heading and encoder position
        odometry.update(Rotation2d.fromDegrees(getHeading()), modules[0].getCurrentState(),
                modules[1].getCurrentState(),
                modules[2].getCurrentState(), modules[3].getCurrentState());

        // SmartDashboard.putNumber("Odometry X",
        // odometry.getPoseMeters().getTranslation().getX());
        // SmartDashboard.putNumber("Odometry Y",
        // odometry.getPoseMeters().getTranslation().getY());;
        // SmartDashboard.putNumber("Raw gyro angle", gyro.getAngle());
        SmartDashboard.putNumber("Robot Heading", getHeading());
        fieldOriented = SmartDashboard.getBoolean("Field Oriented", true);
        // SmartDashboard.putNumber("Gyro Compass Heading", gyro.getCompassHeading());
        // SmartDashboard.putNumber("Compass Offset", compassOffset);
        // SmartDashboard.putBoolean("Current Magnetic Field Disturbance",
        // gyro.isMagneticDisturbance());
    }

    @Override
    public void setOdometry(SwerveDriveOdometry odometry) {
        this.odometry = odometry;
    }

    @Override
    public SwerveDriveOdometry getOdometry() {
        return odometry;
    }

    public void resetOdometry() {
        odometry.resetPosition(new Pose2d(), Rotation2d.fromDegrees(0));
        gyro.reset();
    }

    public double getHeading() {
        double x = gyro.getAngle();
        if (fieldOriented) { //TODO: field oriented
            x -= SmartDashboard.getNumber("Field Offset from North (degrees)", 0);
        }
        return Math.IEEEremainder(x * (isGyroReversed ? -1.0 : 1.0), 360);
    }

    // Resets the gyro, so that the direction the robotic currently faces is
    // considered "forward"
    public void resetHeading() {
        gyro.reset();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void drive(double forward, double strafe, double rotation) {
        //System.err.println(rotation);
        drive(getSwerveStates(forward, strafe, rotation));
    }

    @Override
    public void drive(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeed);

        // Move the modules based on desired (normalized) speed, desired angle, max
        // speed, drive modifier, and whether or not to reverse turning.
        for (int i = 0; i < 4; i++) {
            moduleStates[i] = SwerveModuleState.optimize(moduleStates[i],
                    Rotation2d.fromDegrees(modules[i].getModuleAngle()));
            modules[i].move(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
        }
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(modules[0].getCurrentState(), modules[1].getCurrentState(),
                modules[2].getCurrentState(), modules[3].getCurrentState());
    }

    /**
     * Constructs and returns a ChassisSpeeds objects using forward, strafe, and
     * rotation values.
     * 
     * @param forward  The desired forward speed, in m/s.
     * @param strafe   The desired strafe speed, in m/s.
     * @param rotation The desired rotation speed, in rad/s.
     * @return A ChassisSpeeds object.
     */
    private ChassisSpeeds getChassisSpeeds(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds;
        if (fieldOriented) { //TODO: field oriented
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()).rotateBy(new Rotation2d(Math.PI)));
            //speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Rotation2d.fromDegrees(getHeading()).rotateBy(DriverStation.getAlliance() == Alliance.Blue ? new Rotation2d(Math.PI) : new Rotation2d()));
        } else {
            speeds = new ChassisSpeeds(-1 * forward, -1 * strafe, rotation); //Forward and Strafe are switched to negative so forward is facing intake (robot oreinted specific)
        }
        return speeds;
    }

    /**
     * Constructs and returns four SwerveModuleState objects, one for each side,
     * using forward, strafe, and rotation values.
     * 
     * @param forward  The desired forward speed, in m/s.
     * @param strafe   The desired strafe speed, in m/s.
     * @param rotation The desired rotation speed, in rad/s.
     * @return A SwerveModuleState array, one for each side of the drivetrain (FL,
     *         FR, etc.).
     */
    private SwerveModuleState[] getSwerveStates(double forward, double strafe, double rotation) {
        return kinematics.toSwerveModuleStates(getChassisSpeeds(forward, -strafe, -rotation));
    }

    public void toggleMode() {
        for (SwerveModule module: modules)
            module.toggleMode();
    }

    public void brake() {
        for (SwerveModule module: modules)
            module.brake();
    }

    public void coast() {
        for (SwerveModule module: modules)
            module.coast();
    }

    @Override
    public double getMaxAccelMps2() {
        return Constants.DriveConstants.autoMaxAccelMps2;
    }

    @Override
    public double getMaxSpeedMps() {
        return Constants.DriveConstants.autoMaxSpeedMps;
    }

    @Override
    public double getHeadingDeg() {
        return getHeading();
    }

    @Override
    public void stop() {
        for(SwerveModule module: modules) module.move(0, 0);
    }

    @Override
    public double[][] getPIDConstants() {
        return new double[][] {
            Constants.DriveConstants.xPIDController,
            Constants.DriveConstants.yPIDController,
            Constants.DriveConstants.thetaPIDController
        };
    }

    
}
