package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.encoder.CTREEncoder;
import frc.robot.utils.swerve.SwerveModuleConstants;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
    public static String canbus = "rio";

    /** All joystick, button, and axis IDs. */
    public static class ControllerConstants {
        public static final double axisDeadzone = 0.05;

        public static final int driverControllerID = 0;
        public static final int operatorControllerID = 1;

        // Prevent from acclerating/decclerating to quick
        public static final SlewRateLimiter xDriveLimiter = new SlewRateLimiter(4);
        public static final SlewRateLimiter yDriveLimiter = new SlewRateLimiter(4);
        public static final SlewRateLimiter rotationLimiter = new SlewRateLimiter(4);
    }

    /** All swerve constants. */
    public static class SwerveConstants {
        /** Constants that apply to the whole drive train. */
        public static final double wheelBaseWidth = Units.inchesToMeters(28); // Width of the drivetrain measured from the middle of the wheels.
        public static final double wheelBaseLength = Units.inchesToMeters(28); // Length of the drivetrain measured from the middle of the wheels.
        public static final double wheelDiameter = Units.inchesToMeters(3.75);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBaseLength / 2.0, wheelBaseWidth / 2.0),
            new Translation2d(wheelBaseLength / 2.0, -wheelBaseWidth / 2.0),
            new Translation2d(-wheelBaseLength / 2.0, wheelBaseWidth / 2.0),
            new Translation2d(-wheelBaseLength / 2.0, -wheelBaseWidth / 2.0)
        );

        public static final double driveGearRatio = 6.75 / 1.0; // 6.75:1
        public static final double driveRotationsToMeters = wheelCircumference / driveGearRatio;
        public static final double driveRPMToMPS = driveRotationsToMeters / 60.0;
        public static final double angleGearRatio = 21.43 / 1.0; // 21.43:1
        public static final double angleRPMToRPS = angleGearRatio / 60;

        /** Speed ramp. */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /** Current limiting. */
        public static final int driveCurrentLimit = 40;
        public static final int angleCurrentLimit = 40;

        /** Drive motor PID values. */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /** Drive motor characterization. */
        public static final double driveKS = 0.11937;
        public static final double driveKV = 2.6335;
        public static final double driveKA = 0.46034;

        /** Angle motor PID values. */
        public static final double angleKP = 1.5;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.1;
        public static final double angleKF = 0.0;
        
        /** Swerve constraints. */
        public static final double maxDriveVelocity = 2;
        public static final double maxAngularVelocity = 4;
        public static final double sensitivity = 1;

        /** Inversions. */
        public static final boolean driveInversion = true;
        public static final boolean angleInversion = true;
        public static final boolean encoderInversion = false;

        /** Idle modes. */
        public static final boolean driveBreakMode = false;
        public static final boolean angleBreakMode = false;

        /** 
         * Module specific constants.
         * CanCoder offset is in DEGREES, not radians like the rest of the repo.
         * This is to make offset slightly more accurate and easier to measure.
         */
        public static final SwerveModuleConstants mod0Constants = new SwerveModuleConstants( // FL -x +y
            3,
            4,
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            new CTREEncoder(2, DeviceConfigs.SwerveModuleConfigs.getCANCoder()),
            91.40
        );

        public static final SwerveModuleConstants mod1Constants = new SwerveModuleConstants( // FR +x +y
            5,
            6,
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            new CTREEncoder(3, DeviceConfigs.SwerveModuleConfigs.getCANCoder()),
            128.49
        );

        public static final SwerveModuleConstants mod2Constants = new SwerveModuleConstants( // BL -x -y
            1,
            2,
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            new CTREEncoder(1, DeviceConfigs.SwerveModuleConfigs.getCANCoder()),
            134.4
        );

        public static final SwerveModuleConstants mod3Constants = new SwerveModuleConstants( // BR +x -y
            7,
            8,
            DCMotor.getKrakenX60(1),
            DCMotor.getKrakenX60(1),
            new CTREEncoder(4, DeviceConfigs.SwerveModuleConfigs.getCANCoder()),
            75.49
        );
    }

    public static class kAuto {
        /** PID Values. */
        
        /** Constraints. */
        public static final double maxDriveVelocity = 2.0;
        public static final double maxAngleVelocity = 5.0;
    }
}
