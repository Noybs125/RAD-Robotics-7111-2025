package frc.robot.subsystems.swerve;

import frc.robot.utils.encoder.Encoder;

public class SwerveModuleConstants {
    public final SwerveMotorConstants driveMotor;
    public final SwerveMotorConstants angleMotor;
    public final Encoder encoder;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(SwerveMotorConstants driveMotor, SwerveMotorConstants angleMotor, Encoder encoder, double canCoderOffsetDegrees) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
