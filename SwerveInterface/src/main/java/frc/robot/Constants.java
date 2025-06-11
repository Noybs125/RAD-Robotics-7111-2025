package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.DrivebaseConfig;
import frc.robot.subsystems.swerve.SwerveModuleConstants;
import frc.robot.subsystems.swerve.modules.SwerveModuleType;
import frc.robot.utils.encoder.CTREEncoder;

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
        public static final DrivebaseConfig drivebaseConfig = DrivebaseConfig.getStormSurge(true);
        /** Constants that apply to the whole drive train. */
        public static final double wheelBaseWidth = drivebaseConfig.width; // Width of the drivetrain measured from the middle of the wheels.
        public static final double wheelBaseLength = drivebaseConfig.length; // Length of the drivetrain measured from the middle of the wheels.
        public static final double wheelDiameter = drivebaseConfig.wheelDiameter;
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

        /** Drive motor characterization. */
        public static final double driveKS = 0.11937;
        public static final double driveKV = 2.6335;
        public static final double driveKA = 0.46034;
        
        /** Swerve constraints. */
        public static final double maxDriveVelocity = 2;
        public static final double maxAngularVelocity = 4;
        public static final double sensitivity = 1;

        /** 
         * Module specific constants.
         * CanCoder offset is in DEGREES, not radians like the rest of the repo.
         * This is to make offset slightly more accurate and easier to measure.
         */
        public static final SwerveModuleConstants mod0Constants = drivebaseConfig.moduleConstants[0];

        public static final SwerveModuleConstants mod1Constants = drivebaseConfig.moduleConstants[1];

        public static final SwerveModuleConstants mod2Constants = drivebaseConfig.moduleConstants[2];

        public static final SwerveModuleConstants mod3Constants = drivebaseConfig.moduleConstants[3];

        /** Motor direction */
        public static final boolean driveInversion = mod0Constants.driveMotor.isCCW;
        public static final boolean angleInversion = mod0Constants.angleMotor.isCCW;

        /** Idle modes */
        public static final boolean driveBreakMode = mod0Constants.driveMotor.isBreakMode;
        public static final boolean angleBreakMode = mod0Constants.angleMotor.isBreakMode;

        /** PID Controllers */
        public static final PIDController drivePID = mod0Constants.driveMotor.pid;
        public static final PIDController anglePID = mod0Constants.angleMotor.pid;

        /** Current limiting. */
        public static final int driveCurrentLimit = 40;
        public static final int angleCurrentLimit = 40;
    }

    public static class kAuto {
        /** PID Values. */
        
        /** Constraints. */
        public static final double maxDriveVelocity = 2.0;
        public static final double maxAngleVelocity = 5.0;
    }
}
