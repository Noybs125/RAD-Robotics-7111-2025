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

/**
 * This class is for defining the configurations of each device (motors, encoders, etc.)
 * Each Subsystem gets its own class to contain configurations for all of its devices
 */
public class DeviceConfigs {

    public static class SwerveModuleConfigs {

        public static CANcoderConfiguration getCANCoder(){
            CANcoderConfiguration config = new CANcoderConfiguration();

            return config;
        }

        public static SparkMaxConfig getSparkMaxDrive(){
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(SwerveConstants.driveInversion);
            SparkBaseConfig idleMode = SwerveConstants.driveBreakMode
                ? config.idleMode(IdleMode.kBrake)
                : config.idleMode(IdleMode.kCoast);
            config.openLoopRampRate(SwerveConstants.openLoopRamp);
            config.closedLoopRampRate(SwerveConstants.closedLoopRamp);
            config.smartCurrentLimit(SwerveConstants.driveCurrentLimit);

            ClosedLoopConfig pidConfig = new ClosedLoopConfig();
            pidConfig.p(SwerveConstants.driveKP);
            pidConfig.i(SwerveConstants.driveKI);
            pidConfig.d(SwerveConstants.driveKD);
            pidConfig.velocityFF(SwerveConstants.driveKF);
        
            AbsoluteEncoderConfig encoderConfig = new AbsoluteEncoderConfig();
            encoderConfig.positionConversionFactor(SwerveConstants.driveRotationsToMeters);
            encoderConfig.velocityConversionFactor(SwerveConstants.driveRPMToMPS);

            config.apply(idleMode);
            config.absoluteEncoder.apply(encoderConfig);
            config.closedLoop.apply(pidConfig);

            return config;
        }

        public static SparkMaxConfig getSparkMaxRotation(){
            SparkMaxConfig config = new SparkMaxConfig();
            // configuration goes here...

            config.inverted(SwerveConstants.angleInversion);
            SparkBaseConfig idleMode = SwerveConstants.angleBreakMode
                ? config.idleMode(IdleMode.kBrake)
                : config.idleMode(IdleMode.kCoast);
            
            config.smartCurrentLimit(SwerveConstants.angleCurrentLimit);

            ClosedLoopConfig pidConfig = new ClosedLoopConfig();
            pidConfig.p(SwerveConstants.angleKP);
            pidConfig.i(SwerveConstants.angleKI);
            pidConfig.d(SwerveConstants.angleKD);
            pidConfig.velocityFF(SwerveConstants.angleKF);

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
