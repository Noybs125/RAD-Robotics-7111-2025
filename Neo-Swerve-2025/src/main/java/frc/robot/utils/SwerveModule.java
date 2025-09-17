package frc.robot.utils;



import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  private SparkMax driveMotor;
  private SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
  private RelativeEncoder driveEncoder;
  private SparkAbsoluteEncoderSim driveEncoderConversion;
  private SparkClosedLoopController drivePID;
  private ClosedLoopConfig drivePIDConfig = new ClosedLoopConfig();
  private SimpleMotorFeedforward driveFeedforward;

  private SparkMax angleMotor;
  private SparkMaxConfig angleMotorConfig = new SparkMaxConfig();
  private RelativeEncoder angleEncoder;
  private SparkAbsoluteEncoderSim angleEncoderConversion;
  private SparkClosedLoopController anglePID;
  private ClosedLoopConfig anglePIDConfig = new ClosedLoopConfig();
  
  private final CANcoder canCoder;
  private final double canCoderOffsetDegrees;
  private final CANcoderConfigurator canConfig;

  private double lastAngle;

  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new SparkMax(constants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);
    drivePID = driveMotor.getClosedLoopController();

    angleMotor = new SparkMax(constants.angleMotorID, MotorType.kBrushless);
    angleEncoder = angleMotor.getEncoder();
    anglePID = angleMotor.getClosedLoopController();

    canCoder = new CANcoder(constants.canCoderID);
    canCoderOffsetDegrees = constants.canCoderOffsetDegrees;
    canConfig = canCoder.getConfigurator();

    configureDevices();
    lastAngle = getState().angle.getRadians();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    // System.out.println("Angle: " + state.angle.getRadians() + "Mod #: " + moduleNumber);
    state = SwerveModuleState.optimize(state, getState().angle);

    if (isOpenLoop) {
      double speed = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      drivePID.setReference(speed, SparkMax.ControlType.kDutyCycle);
    } else {
      drivePID.setReference(state.speedMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    double angle = Math.abs(state.speedMetersPerSecond) <= Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND * 0.01
      ? lastAngle
      : state.angle.getRadians();

    anglePID.setReference(angle, SparkMax.ControlType.kPosition);

    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    double velocity = driveEncoder.getVelocity();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModuleState(velocity, rot);
  }

  public double getCanCoder() {
    return canCoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public Rotation2d getAngle() {
    return new Rotation2d(angleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public Rotation2d getCanCoderDegrees(){
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValueAsDouble() * 360);
  }
  
  private void configureDevices() {
    // CanCoder configuration.
    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    
    canConfig.apply(canCoderConfiguration);
  
    // Drive motor configuration.
    //driveMotorConfig.restoreFactoryDefaults();
    driveMotorConfig.inverted(Constants.kSwerve.DRIVE_MOTOR_INVERSION);
    driveMotorConfig.idleMode(Constants.kSwerve.DRIVE_IDLE_MODE);
    driveMotorConfig.openLoopRampRate(Constants.kSwerve.OPEN_LOOP_RAMP);
    driveMotorConfig.closedLoopRampRate(Constants.kSwerve.CLOSED_LOOP_RAMP);
    driveMotorConfig.smartCurrentLimit(Constants.kSwerve.DRIVE_CURRENT_LIMIT);

    drivePIDConfig.p(Constants.kSwerve.DRIVE_KP);
    drivePIDConfig.i(Constants.kSwerve.DRIVE_KI);
    drivePIDConfig.d(Constants.kSwerve.DRIVE_KD);
    drivePIDConfig.velocityFF(Constants.kSwerve.DRIVE_KF);
 
    //setPositionConversionFactor && setVelocityConversionFactor are in SparkAbsoluteEncoderSim
    AbsoluteEncoderConfig driveEncoderConfig = new AbsoluteEncoderConfig();
    driveEncoderConfig.positionConversionFactor(Constants.kSwerve.DRIVE_ROTATIONS_TO_METERS);
    driveEncoderConfig.velocityConversionFactor(Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND);
    driveEncoder.setPosition(0);

    driveMotorConfig.absoluteEncoder.apply(driveEncoderConfig);
    driveMotorConfig.closedLoop.apply(drivePIDConfig);

    driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    // Angle motor configuration.
    //angleMotorConfig.restoreFactoryDefaults();
    angleMotorConfig.inverted(Constants.kSwerve.ANGLE_MOTOR_INVERSION);
    angleMotorConfig.idleMode(Constants.kSwerve.ANGLE_IDLE_MODE);
    angleMotorConfig.smartCurrentLimit(Constants.kSwerve.ANGLE_CURRENT_LIMIT);

    anglePIDConfig.p(Constants.kSwerve.ANGLE_KP);
    anglePIDConfig.i(Constants.kSwerve.ANGLE_KI);
    anglePIDConfig.d(Constants.kSwerve.ANGLE_KD);
    anglePIDConfig.velocityFF(Constants.kSwerve.ANGLE_KF);

    anglePIDConfig.positionWrappingEnabled(true);
    anglePIDConfig.positionWrappingMaxInput(2 * Math.PI);
    anglePIDConfig.positionWrappingMinInput(0);

    var angleEncoderConfig = new AbsoluteEncoderConfig();
    angleEncoderConfig.positionConversionFactor(Constants.kSwerve.ANGLE_ROTATIONS_TO_RADIANS);
    angleEncoderConfig.velocityConversionFactor(Constants.kSwerve.ANGLE_RPM_TO_RADIANS_PER_SECOND);

    angleMotorConfig.closedLoop.apply(anglePIDConfig);
    angleMotorConfig.absoluteEncoder.apply(angleEncoderConfig);
    angleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    angleEncoder.setPosition(Units.degreesToRadians((canCoder.getAbsolutePosition().getValueAsDouble() * 360) - canCoderOffsetDegrees)); // added ".getValue..."
  }
}
