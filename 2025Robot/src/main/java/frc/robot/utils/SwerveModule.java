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
import frc.robot.Constants;

public class SwerveModule {
  public final int moduleNumber;

  public final TalonFX driveMotor;
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

  /**
   * Constructor for SwerveModule class.
   * Creates drive motor object.
   * Creates angle motor object.
   * creats feedforward object for the motor.
   * Creates CANCoder object.
   * @param moduleNumber -Type "int", used to specify module number properties for the objects.
   * @param constants -Type "SwerveModuleConstants", used to allow the objects to access the motor constants.
   */
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
  /**
   * Sets the state of the swerve module. 
   * Uses the SwerveModuleState objects.
   * @param state -Type "SwerveModuleState", it is an object defining what state the module is in.
   * @param isOpenLoop -Type "boolean", true if is open loop, false if otherwise.
   * @see https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModuleState.html.
   */
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

  /**
   * Constructs a new SwerveModuleState object.
   * Uses getVelocity, fromRotations, and SwerveModuleState methods.
   * @return -Type "SwerveModuleState", SwerveModuleState object created based off of velocity and rotation
   * @see -Link to getVelocity: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getVelocity().
   * @see -Link to fromRotations: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation2d.html#fromRotations(double).
   * @see -Link to SwerveModuleState: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModuleState.html#%3Cinit%3E(double,edu.wpi.first.math.geometry.Rotation2d).
   */
  public SwerveModuleState getState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble() * 60 * Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(velocity, rot);
  }

  /**
   * Constructs a new SwerveModuleState object, but inverts the velocity, inverting the state.
   * Uses getVelocity, fromRotations, and SwerveModuleState methods.
   * @return -Type "SwerveModuleState", SwerveModuleState object created based off of velocity and rotation
   * @see -Link to getVelocity: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getVelocity().
   * @see -Link to fromRotations: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation2d.html#fromRotations(double).
   * @see -Link to SwerveModuleState: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModuleState.html#%3Cinit%3E(double,edu.wpi.first.math.geometry.Rotation2d).
   */
  public SwerveModuleState getInvertedState() {
    double velocity = driveMotor.getVelocity().getValueAsDouble() * 60 * Constants.kSwerve.DRIVE_RPM_TO_METERS_PER_SECOND;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModuleState(-velocity, rot.unaryMinus());
  }

  /**
   * Uses getAbsolutePosition method to get the position of the canCoder.
   * @return -Type "double", gets the canCoder absolute position.
   * @see -Link to getAbsolutePosition method: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getAbsolutePosition().
   */
  public double getCanCoder() {
    return canCoder.getAbsolutePosition().getValueAsDouble();
  }

  /**
   * gets the angle of the motor from the motor encoder.
   * @return Type "Rotation2d", gives a rotation2d object containing the motor rotation as an angle. Unit is rotation.
   * @see -Link to getPosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getPosition().
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
  }

  /**
   * Uses getPosition and fromRotations to find distance and rotation to supply SwerveModulePosition.
   * @return -Type "SwerveModulePosition", SwerveModulePosition object based off of the distance of the motor and the rotation
   * @see -Link to getPosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getPosition()
   * @see -Link to fromRotations: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation2d.html#fromRotations(double).
   * @see -Link to SwerveModulePosition: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModulePosition.html#%3Cinit%3E(double,edu.wpi.first.math.geometry.Rotation2d).
   */
  public SwerveModulePosition getPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * Constants.kSwerve.WHEEL_CIRCUMFERENCE;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(distance, rot);
  }

  /**
   * Uses getPosition and fromRotations to find distance and rotation to supply SwerveModulePosition. Inverts rotation.
   * @return -Type "SwerveModulePosition", SwerveModulePosition object based off of the distance of the motor and the rotation
   * @see -Link to getPosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getPosition()
   * @see -Link to fromRotations: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/geometry/Rotation2d.html#fromRotations(double).
   * @see -Link to SwerveModulePosition: https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/kinematics/SwerveModulePosition.html#%3Cinit%3E(double,edu.wpi.first.math.geometry.Rotation2d).
   */
  public SwerveModulePosition getInvertedPosition() {
    double distance = driveMotor.getPosition().getValueAsDouble() * Constants.kSwerve.WHEEL_CIRCUMFERENCE;
    Rotation2d rot = Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    return new SwerveModulePosition(distance, rot.unaryMinus());
  }

  /**
   * Gets the angle of the CanCoder using absolute position
   * Uses getAbsolutePosition method.
   * @return -Type "Rotation2d", assignes the rotation value to a rotation 2d object as properties.
   * @see -Link to getAbsolutePosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreCANcoder.html#getAbsolutePosition().
   */
  public Rotation2d getCanCoderDegrees(){
    return Rotation2d.fromDegrees(canCoder.getAbsolutePosition().getValueAsDouble() * 360);
  }
  
  /**
   * Configures all of the devices in this class, including the constants.
   * Uses setPosition and getConfigurator methods, along with the CANcoderConfiguration class.
   * @see -Link to setPosition: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#setPosition(edu.wpi.first.units.measure.Angle).
   * @see -Link to getConfigurator: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/hardware/core/CoreTalonFX.html#getConfigurator().
   * @see -Link to CANcoderConfiguration: https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/CANcoderConfiguration.html.
   */
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
