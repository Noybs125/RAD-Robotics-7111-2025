package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.utils.SwerveModuleConstants;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class contains values that remain constant while the robot is running.
 * 
 * It's split into categories using subclasses, preventing too many members from
 * being defined on one class.
 */
public class Constants {
  public static class kVision {
    // TODO: change variable names on actual robot
    public static final Transform3d cameraToRobotCenter2 = new Transform3d(
        Units.inchesToMeters(-5.625), 
        Units.inchesToMeters(11.5), 
        Units.inchesToMeters(11.895), 
        new Rotation3d(0, 0, Units.degreesToRadians(-22.8)));
    public static final Transform3d cameraToRobotCenter1 = new Transform3d(
        Units.inchesToMeters(-6.5), 
        Units.inchesToMeters(-4.75), 
        0, 
        new Rotation3d(0, 0.0873, 3.1416));

    public static final double cameraHeight = Units.inchesToMeters(0);

    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 1;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(), 1, 1, 1 * Math.PI);
  }
  
  public static class kSimulation {
    public static final PIDController elevatorPid = new PIDController(85, 6,3);
    public static final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(1.23, 0.36, 7.22, 0.05);

    public static final PIDController wristPid = new PIDController(1, 0, 0.05);
    public static final ArmFeedforward wristFF = new ArmFeedforward(0.1, 0.1, 0.1);

    public static final double elevatorSimGearRatio = 1;    

    public static final double elevatorDrumRadius = Units.inchesToMeters(2.0);
    public static final double elevatorEncoderDistPerPulse =
      2.0 * Math.PI * elevatorDrumRadius / 4096;
      
    public static final int encoderAChannel = 0;
    public static final int encoderBChannel = 1;

    // TODO Adjust these values below
    public static final double minHeight = 0.14925;
    public static final double maxHeight = 1.864;
    public static final double startingHeight = 1.5;

    public static final double[] measurementStdDevs = {};

    public static final ElevatorSim elevatorSimConstants = new ElevatorSim(elevatorFF.getKv(), elevatorFF.getKa(), DCMotor.getKrakenX60(2), minHeight, maxHeight, true, minHeight, measurementStdDevs);
    
    public static final SingleJointedArmSim armSim = new SingleJointedArmSim(DCMotor.getKrakenX60(1), 44, 0.93573, 0.532, Units.degreesToRadians(55), Units.degreesToRadians(215), true, Units.degreesToRadians(55), measurementStdDevs);

  }
  /** All joystick, button, and axis IDs. */
  public static class kControls {
    public static final double AXIS_DEADZONE = 0;

    public static final int LEFT_JOY_ID = 0;
    public static final int RIGHT_JOY_ID = 1;
    public static final int DRIVE_JOYSTICK_ID = 2;

    public static final int TRANSLATION_X_AXIS = Joystick.AxisType.kX.value;
    public static final int TRANSLATION_Y_AXIS = Joystick.AxisType.kY.value;
    public static final int ROTATION_AXIS = Joystick.AxisType.kX.value;
    public static boolean isFieldRel;

    public static final int GYRO_RESET_BUTTON = XboxController.Button.kY.value;

    // Prevent from accelerating/decelerating to quick
    public static final SlewRateLimiter X_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter Y_DRIVE_LIMITER = new SlewRateLimiter(4);
    public static final SlewRateLimiter THETA_DRIVE_LIMITER = new SlewRateLimiter(4);
  }

  /** All swerve constants. */
  public static class kSwerve {
    /* Constants that apply to the whole drive train. */
    public static final double TRACK_WIDTH = Units.inchesToMeters(21.25); // Width of the drivetrain measured from the middle of the wheels.
    public static final double WHEEL_BASE = Units.inchesToMeters(23.25); // Length of the drivetrain measured from the middle of the wheels.
    public static final double MODULE_TO_CENTER = Units.inchesToMeters(13.95); // Distance from the center of the module to the center of the robot
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
      new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0)
    );

    public static final double DRIVE_GEAR_RATIO = 6.11 / 1.0; // 6.75:1
    public static final double DRIVE_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
    public static final double DRIVE_RPM_TO_METERS_PER_SECOND = DRIVE_ROTATIONS_TO_METERS / 60.0;
    public static final double ANGLE_GEAR_RATIO = ((468.0 / 35.0) / 1.0); // 21.43:1
    public static final double ANGLE_ROTATIONS_TO_RADIANS = (Math.PI * 2) / ANGLE_GEAR_RATIO;
    public static final double ANGLE_RPM_TO_RADIANS_PER_SECOND = ANGLE_ROTATIONS_TO_RADIANS / 60; // DRIVE_ROTATIONS_TO_METERS / 60.0;

    /* Speed ramp. */
    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /* Current limiting. */
    public static final int DRIVE_CURRENT_LIMIT = 40;
    public static final int DRIVE_CURRENT_THRESHOLD = 60;
    public static final int ANGLE_CURRENT_LIMIT = 20;
    public static final int ANGLE_CURRENT_THRESHOLD = 40;

    /* Drive motor PID values. */
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KF = 0.0;

    /* Drive motor characterization. */
    public static final double DRIVE_KS = 0.11937;
    public static final double DRIVE_KV = 2.3;
    public static final double DRIVE_KA = 0.11;

    /* Angle motor PID values. */
    public static final double ANGLE_KP = 40;
    public static final double ANGLE_KI = 5;
    public static final double ANGLE_KD = 0;
    public static final double ANGLE_KF = 0;
  
    /* Swerve constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 6;

    /* Inversions. */
    public static final InvertedValue DRIVE_MOTOR_INVERSION = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue ANGLE_MOTOR_INVERSION = InvertedValue.CounterClockwise_Positive;
    public static final boolean CANCODER_INVERSION = false;

    /* Idle modes. */
    public static final NeutralModeValue DRIVE_IDLE_MODE = NeutralModeValue.Brake;
    public static final NeutralModeValue ANGLE_IDLE_MODE = NeutralModeValue.Coast;
    /** 
     * Module specific constants.
     * CanCoder offset is in DEGREES, not radians like the rest of the repo.
     * This is to make offset slightly more accurate and easier to measure.
     */
    public static final SwerveModuleConstants MOD_0_Constants = new SwerveModuleConstants( // FR +x +y
      12,
      3,
      0,
      0.032,
      true
    );
    
    public static final SwerveModuleConstants MOD_1_Constants = new SwerveModuleConstants( // FL -x +y
      9,
      10,
      3,
      -0.371,
      true
    );

    
    public static final SwerveModuleConstants MOD_2_Constants = new SwerveModuleConstants( // BL -x -y
      7,
      6,
      2,
      0.486,
      true
    );

    public static final SwerveModuleConstants MOD_3_Constants = new SwerveModuleConstants( // BR +x -y
      11,
      4,
      1,
      -0.123,
      true
    );
  }

  public static class kMechanisms {
    /* Current Limiting */
    public static final double wristCurrentLimit = 40;
    public static final double elevatorCurrentLimit = 40;
    public static final double flywheelCurrentLimit = 40;
    public static final double elevatorWinchDiameter = 2;
    public static final double wristGearRatio = 48.13;
    /** Elevator max speed allowed, in percent. 0 = 0%, 1 = 100% */
    public static final double elevatorMaxSpeed = 20;
    /** Elevator Max Height in feet */
    public static final double elevatorMaxHeight = 8;

    /** Minimum safe height for the elevator so that the wrist can move to  mechanical limits, in scale of percent of max height */
    public static final double elevatorMinSafeWristHeight = (1+(2/3)) / elevatorMaxHeight; //currently set to 20 inches high, theoretical limit is 16 inches
    /** Height of elevator that it will not allow the wrist to point past precautionary limits */
    public static final double elevatorPrecautionWristHeight = elevatorMinSafeWristHeight + 0.1;
    
    /** Maximum height the elevator can be before restricting the wrist max rotation speed */
    public static final double elevatorMaxSafeWristHeight = 0.65; //currently set to 5.2 ft

    /** Max height elevator will be allowed to move full speed */
    public static final double elevatorMaxSafeHeight = 0.9; //currently set to 7.2 ft

    // Safe rotation range at any height is between these two variables, min being away from deep climb module and max being towards it.
    /** Min safe rotation for the wrist before the elevator could let it hit the robot, going past this variable (in rotations) is a risk for hitting the robot's frame */
    public static final double wristMinSafeRotation = -0.25; //-90 digrees of rotation
    public static final double wristPrecautionaryMinRotation = -0.3; //108 digrees of rotation
    /** Max safe rotation for the wrist before the elevator could let it hit the robot, going past this variable (in rotations) is a risk for hitting the deep climb module or robot's frame */
    public static final double wristMaxSafeRotation = 0.15 ; //54 digrees of rotation
    public static final double wristPrecautionaryMaxRotation = 0.15; //54 digrees of rotation

    // Multiplier for the maximum speed of the wrist and elevator, in percent of max speed
    public static final double maxWristReducedSpeed = 0.30; //30% currently
    public static final double maxElevReducedSpeed = 0.30; //30% currently

    //mechanical limits of elevator, multiply by elevatorMaxHeight to get the height of the robot in feet
    public static final double elevatorMaxPosition = 1.15;
    public static final double elevatorMinPosition = 0.01;

    //mechanical limits of wrist in rotations
    /**Maximum mechanical limit of wrist in rotation */
    public static final double maxWristPosition = -0.25; //90 digrees toward deep climb module
    /** Minimum mechanical limit of wrist in rotation */
    public static final double minWristPosition = 0.5; //180 digrees away from deep climb module

    public static final double maxWristSpeed = 5;
    public static final SimpleMotorFeedforward wristFF = null;//new SimpleMotorFeedforward(0.12, 5.78, 0.08);
    public static final SimpleMotorFeedforward deepClimbFF = null;

    public static PIDController elevatorPID = new PIDController(34, 0.65, 0.0);
    public static PIDController armPID = new PIDController(25, 0.0, 0.0); // - 15, 0.03, 0.01?
    public static PIDController deepClimbPID = new PIDController(0, 0, 0);

    /** Motor Configurations */

    public static TalonFXConfiguration elevator1Config(){
      TalonFXConfiguration elevator1Config = new TalonFXConfiguration();
      
      elevator1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      elevator1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      return elevator1Config;
    }

    public static TalonFXConfiguration elevator2Config(){
      TalonFXConfiguration elevator2Config = new TalonFXConfiguration();
      
      elevator2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      elevator2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      return elevator2Config;
    }

    public static TalonFXConfiguration wristConfig(){
      TalonFXConfiguration wristConfig = new TalonFXConfiguration();
      wristConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      return wristConfig;
    }
  }

  public static class kAuto {
    public static final double rightOffset = 0.11;
    public static final double leftOffset = 0.16;
    public static final double centerOffset = 0.05;

    /* PID Values. */
    public static final double X_CONTROLLER_KP = 1.0;
    public static final double Y_CONTROLLER_KP = 1.0;
    public static final double THETA_CONTROLLER_KP = 1.0;
    
    /* Constraints. */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.41;
    public static final double MAX_ACCEL_METERS_PER_SECOND_SQUARED = 5.41;
    public static final double MAX_ANGULAR_RADIANS_PER_SECOND = 6.99;
    public static final double MAX_ANGULAR_ACCEL_RAD_PER_SECOND = 12.56;
    /* Pathplanner config */

    public static final PathConstraints constraints = new PathConstraints(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCEL_METERS_PER_SECOND_SQUARED, MAX_ANGULAR_RADIANS_PER_SECOND, MAX_ANGULAR_ACCEL_RAD_PER_SECOND);
    public static final PathConstraints reefConstraints = new PathConstraints(2, 1.25, 2, 2);

    public static final int numModules = 4;
    public static final double massKgs = Units.lbsToKilograms(138);
    public static final double MOI = massKgs * (Units.inchesToMeters(28) * Units.inchesToMeters(30));
    public static final ModuleConfig moduleConfig = new ModuleConfig(kSwerve.WHEEL_DIAMETER / 2, MAX_VELOCITY_METERS_PER_SECOND,
        0.8, DCMotor.getKrakenX60(1), kSwerve.DRIVE_GEAR_RATIO, kSwerve.DRIVE_CURRENT_LIMIT, 1);


    public static final Translation2d[] moduleLocations = new Translation2d[] {
      new Translation2d(kSwerve.WHEEL_BASE / 2.0, -kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(-kSwerve.WHEEL_BASE / 2.0, -kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(kSwerve.WHEEL_BASE / 2.0, kSwerve.TRACK_WIDTH / 2.0),
      new Translation2d(-kSwerve.WHEEL_BASE / 2.0, kSwerve.TRACK_WIDTH / 2.0)
    };

    public static final RobotConfig config = new RobotConfig(massKgs, MOI, moduleConfig, moduleLocations);

    public static final PPHolonomicDriveController cont = new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    //new PIDConstants(13, 0.0001, 0.022), // Translation PID constants
                    //new PIDConstants(9.15, 0.09, 0.9) // Rotation PID constants
                    new PIDConstants(10, 0.0, 0.03), // Translation PID constants
                    new PIDConstants(5, 0.0, 0.03)    // Rotation PID constants
            );

    
  }

}
