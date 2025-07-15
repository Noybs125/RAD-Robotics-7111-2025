package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.config.DrivebaseConfig;
import frc.robot.subsystems.swerve.config.SwerveModuleConfig;
import frc.robot.subsystems.swerve.config.SwerveMotorConfig;

/**
 * This class is for defining the configurations of each device (motors, encoders, etc.)
 */
public class DeviceConfigs {

    public static class StormSurgeConfigs {
        public static TalonFXConfiguration getSwerveModuleDriveMotor(SwerveMotorConfig motorConfig){
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
            config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
            
            config.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
            config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;

            config.CurrentLimits.SupplyCurrentLimitEnable = true;

            config.Slot0.kP = motorConfig.pid.getP();
            config.Slot0.kI = motorConfig.pid.getI();
            config.Slot0.kD = motorConfig.pid.getD();

            config.Slot0.kS = 0.11937;
            config.Slot0.kV = 2.3;
            config.Slot0.kA = 0.11;

            config.Feedback.SensorToMechanismRatio = motorConfig.gearRatio;

            config.MotorOutput.NeutralMode = motorConfig.isBreakMode
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
            config.MotorOutput.Inverted = motorConfig.isCCW
                ? InvertedValue.CounterClockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

            return config;
        }

        public static TalonFXConfiguration getSwerveModuleAngleMotor(){
            return null;
        }

        public static CANcoderConfiguration getSwerveModuleEncoder(){
            return null;
        }
        
    }

    public static class SwerveModuleConfigs {

        public static CANcoderConfiguration getCANCoder(){
            CANcoderConfiguration config = new CANcoderConfiguration();

            return config;
        }

        public static SparkMaxConfig getSparkMaxDrive(SwerveMotorConfig constants){
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(constants.isCCW);
            SparkBaseConfig idleMode = constants.isBreakMode
                ? config.idleMode(IdleMode.kBrake)
                : config.idleMode(IdleMode.kCoast);
            config.openLoopRampRate(SwerveConstants.openLoopRamp);
            config.closedLoopRampRate(SwerveConstants.closedLoopRamp);
            config.smartCurrentLimit(constants.currentLimit);

            ClosedLoopConfig pidConfig = new ClosedLoopConfig();
            pidConfig.p(constants.pid.getP());
            pidConfig.i(constants.pid.getI());
            pidConfig.d(constants.pid.getD());
            pidConfig.velocityFF(0.1);
        
            AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
            encoderConfig.positionConversionFactor(SwerveConstants.driveRotationsToMeters);
            encoderConfig.velocityConversionFactor(SwerveConstants.driveRPMToMPS);

            config.apply(idleMode);
            config.absoluteEncoder.apply(encoderConfig);
            config.closedLoop.apply(pidConfig);

            return config;
        }

        public static SparkMaxConfig getSparkMaxRotation(SwerveMotorConfig constants){
            SparkMaxConfig config = new SparkMaxConfig();
            // configuration goes here...

            config.inverted(constants.isCCW);
            SparkBaseConfig idleMode = constants.isBreakMode
                ? config.idleMode(IdleMode.kBrake)
                : config.idleMode(IdleMode.kCoast);
            
            config.smartCurrentLimit(constants.currentLimit);

            ClosedLoopConfig pidConfig = new ClosedLoopConfig();
            pidConfig.p(constants.pid.getP());
            pidConfig.i(constants.pid.getI());
            pidConfig.d(constants.pid.getD());
            pidConfig.velocityFF(0.1);

            pidConfig.positionWrappingEnabled(true);
            pidConfig.positionWrappingMaxInput(2 * Math.PI);
            pidConfig.positionWrappingMinInput(0);

            var encoderConfig = new AbsoluteEncoderConfig();
            encoderConfig.positionConversionFactor(1 / SwerveConstants.angleGearRatio);
            encoderConfig.velocityConversionFactor(1 / SwerveConstants.angleGearRatio);

            config.apply(idleMode);
            config.closedLoop.apply(pidConfig);
            config.absoluteEncoder.apply(encoderConfig);

            return config;
        }

        public static TalonFXConfiguration getTalonFXDrive(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            
            config.MotorOutput.Inverted = SwerveConstants.driveInversion
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
            config.MotorOutput.NeutralMode = SwerveConstants.driveBreakMode
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

            return config;
        }

        public static TalonFXConfiguration getTalonFXRotation(){
            TalonFXConfiguration config = new TalonFXConfiguration();
            
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            return config;
        }

        public static CANcoderConfiguration getCANcoder(){
            CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
            canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
            canCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            return canCoderConfiguration;
        }
    }
}
