package frc.robot.utils.swerve;

import frc.robot.utils.encoder.Encoder;

public class SwerveModuleConstants {
    public final MotorConstants driveMotor;
    public final MotorConstants angleMotor;
    public final Encoder encoder;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(MotorConstants driveMotor, MotorConstants angleMotor, Encoder encoder, double canCoderOffsetDegrees) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
