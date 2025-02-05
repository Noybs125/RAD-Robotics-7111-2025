package frc.robot.utils;


import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  private final TalonFX driveMotor;
  private final SimpleMotorFeedforward driveFeedforward;
  private final TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
  private boolean driveMotorInversion;
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);

  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
//Change back to private after
  public final TalonFX angleMotor;
  private final PositionVoltage anglePosition = new PositionVoltage(0);

  TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
  
  private final CANcoder canCoder;
  private final double canCoderOffsetRotations;
  private final CANcoderConfigurator canConfig;

  // Constructor
  public SwerveModule(int moduleNumber, SwerveModuleConstants constants) {
    this.moduleNumber = moduleNumber;
    
    driveMotor = new TalonFX(constants.driveMotorID);
    driveMotorInversion = constants.isInverted;
    driveFeedforward = new SimpleMotorFeedforward(Constants.kSwerve.DRIVE_KS, Constants.kSwerve.DRIVE_KV, Constants.kSwerve.DRIVE_KA);

    angleMotor = new TalonFX(constants.angleMotorID);

    canCoder = new CANcoder(constants.canCoderID);
    canCoderOffsetRotations = constants.canCoderOffsetRotations;
    canConfig = canCoder.getConfigurator();

    configureDevices();
  }

  public void setState(SwerveModuleState state, boolean isOpenLoop) {
    // Prevents angle motor from turning further than it needs to. 
    // E.G. rotating from 10 to 270 degrees CW vs CCW.
    // System.out.println("Angle: " + state.angle.getRadians() + "Mod #: " + moduleNumber);

   
    state.optimize(getState().angle);


    if (isOpenLoop) {
      driveDutyCycle.Output = state.speedMetersPerSecond / Constants.kSwerve.MAX_VELOCITY_METERS_PER_SECOND;
      driveMotor.setControl(driveDutyCycle);
      
    } else {
      driveVelocity.Velocity = state.speedMetersPerSecond / Constants.kSwerve.WHEEL_CIRCUMFERENCE;
      //driveVelocity.FeedForward = driveFeedforward.calculate(state.speedMetersPerSecond);
      driveMotor.setControl(driveVelocity);
    }

    angleMotor.setControl(anglePosition.withPosition(state.angle.getRotations()));

  }

  public SwerveModuleState getState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble() * 60 * Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(velocity, rot);
  }

  public double getCanCoder() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * Constants.kSwerve.WHEEL_CIRCUMFERENCE;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(distance, rot);
  }
  public SwerveModulePosition getInvertedPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * Constants.kSwerve.WHEEL_CIRCUMFERENCE;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(distance, rot.unaryMinus());
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

    

    driveMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.kSwerve.OPEN_LOOP_RAMP;
    driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.kSwerve.CLOSED_LOOP_RAMP;
    
    driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.kSwerve.OPEN_LOOP_RAMP;
    driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.kSwerve.CLOSED_LOOP_RAMP;

    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kSwerve.DRIVE_CURRENT_LIMIT; // change the constant
    driveMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.kSwerve.DRIVE_CURRENT_LIMIT;

    driveMotorConfig.Slot0.kS = Constants.kSwerve.DRIVE_KS;
    driveMotorConfig.Slot0.kV = Constants.kSwerve.DRIVE_KV;
    driveMotorConfig.Slot0.kA = Constants.kSwerve.DRIVE_KA;

    driveMotorConfig.MotorOutput.Inverted = driveMotorInversion
    ? InvertedValue.CounterClockwise_Positive 
    : InvertedValue.Clockwise_Positive;

    driveMotorConfig.MotorOutput.NeutralMode = Constants.kSwerve.DRIVE_IDLE_MODE;
    
    driveMotorConfig.Slot0.kP = Constants.kSwerve.DRIVE_KP;
    driveMotorConfig.Slot0.kI = Constants.kSwerve.DRIVE_KI;
    driveMotorConfig.Slot0.kD = Constants.kSwerve.DRIVE_KD;

    driveMotorConfig.Feedback.SensorToMechanismRatio = Constants.kSwerve.DRIVE_GEAR_RATIO;
 
    driveMotor.setPosition(0);

    driveMotor.getConfigurator().apply(driveMotorConfig);    

    // Angle motor configuration.

    angleMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.kSwerve.ANGLE_CURRENT_LIMIT;
    angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    angleMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
    //angleMotorConfig.Feedback.FeedbackRemoteSensorID = canCoder.getDeviceID();
  
    angleMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    angleMotorConfig.MotorOutput.NeutralMode = Constants.kSwerve.ANGLE_IDLE_MODE;    
    
    
    angleMotorConfig.Slot0.kP = Constants.kSwerve.ANGLE_KP;
    angleMotorConfig.Slot0.kI = Constants.kSwerve.ANGLE_KI;
    angleMotorConfig.Slot0.kD = Constants.kSwerve.ANGLE_KD;

    angleMotorConfig.Feedback.SensorToMechanismRatio = Constants.kSwerve.ANGLE_GEAR_RATIO;

    angleMotor.getConfigurator().apply(angleMotorConfig);
    
    angleMotor.setPosition((-canCoder.getAbsolutePosition().getValueAsDouble() + canCoderOffsetRotations)); // added ".getValue..." 
  }
}
