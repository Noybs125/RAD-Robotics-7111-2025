package frc.robot.subsystems.swerve.config;

import frc.robot.utils.encoder.GenericEncoder;

public class SwerveModuleConfig {
    public final SwerveMotorConfig driveMotor;
    public final SwerveMotorConfig angleMotor;
    public final GenericEncoder encoder;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConfig(SwerveMotorConfig driveMotor, SwerveMotorConfig angleMotor, GenericEncoder encoder, double canCoderOffsetDegrees) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
