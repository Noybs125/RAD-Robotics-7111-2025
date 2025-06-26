package frc.robot.subsystems.swerve;

import frc.robot.utils.encoder.GenericEncoder;

public class SwerveModuleConstants {
    public final SwerveMotorConstants driveMotor;
    public final SwerveMotorConstants angleMotor;
    public final GenericEncoder encoder;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(SwerveMotorConstants driveMotor, SwerveMotorConstants angleMotor, GenericEncoder encoder, double canCoderOffsetDegrees) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
