package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.utils.SwerveModuleConstants;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
  /** All joystick, button, and axis IDs. */
  public static class kControls {
    public static final double AXIS_DEADZONE = 0.05; // zero for path

    public static final int LEFT_JOY_ID = 0;
    public static final int RIGHT_JOY_ID = 1;
    public static final int DRIVE_JOYSTICK_ID = 2;

    public static final int TRANSLATION_X_AXIS = Joystick.AxisType.kX.value;
    public static final int TRANSLATION_Y_AXIS = Joystick.AxisType.kY.value;
    public static final int ROTATION_AXIS = Joystick.AxisType.kX.value;

    public static final int GYRO_RESET_BUTTON = XboxController.Button.kY.value;

    // Prevent from acclerating/decclerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /** Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(28); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(28); // Length of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.75);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final double DRIVE_GEAR_RATIO = 6.75 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = 21.43 / 1.0; // 21.43:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = ANGLE_ROTATIONS_TO_RADIANS / 60; // DRIVE_ROTATIONS_TO_METERS / 60.0;

    /** Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /** Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int ANGLE_CURRENT_LIMIT = 40;

    /** Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /** Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.6335;
    public static final double DRIVE_KA = 0.46034;

    /** Angle motor PID values. */
    public static final double ANGLE_KP = 1.5;
    public static final double ANGLE_KI = 0.0;
    public static final double ANGLE_KD = 0.1;
    public static final double ANGLE_KF = 0.0;
    
    /** Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 4;
    public static final double SENSITIVITY = 1;

    /** Inversions. */
    public static final boolean DRIVE_MOTOR_INVERSION = true;
    public static final boolean ANGLE_MOTOR_INVERSION = true;
    public static final boolean CANCODER_INVERSION = false;

    /** Idle modes. */
    public static final IdleMode DRIVE_IDLE_MODE = IdleMode.kCoast;
    public static final IdleMode ANGLE_IDLE_MODE = IdleMode.kCoast;

    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants( // FL -x +y
      3,
      4,
      2,
      91.40
    );

    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants( // FR +x +y
      5,
      6,
      3,
      128.49
    );

    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants( // BL -x -y
      1,
      2,
      1,
      134.4
    );

    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants( // BR +x -y
      7,
      8,
      4,
      75.49
    );
  }

  public static class kAuto {
    /** PID Values. */
    public static final double X_CONTROLLER_KP = 1.0;
    public static final double Y_CONTROLLER_KP = 1.0;
    public static final double THETA_CONTROLLER_KP = 1.0;
    
    /** Constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 2.0;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5.0;
  }
}
