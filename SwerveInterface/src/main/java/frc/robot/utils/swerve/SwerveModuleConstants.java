package frc.robot.utils.swerve;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.utils.encoder.Encoder;

public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final DCMotor driveMotor;
    public final DCMotor angleMotor;
    public final Encoder encoder;
    public final double canCoderOffsetDegrees;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, DCMotor driveMotor, DCMotor angleMotor, Encoder encoder, double canCoderOffsetDegrees) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.encoder = encoder;
        this.canCoderOffsetDegrees = canCoderOffsetDegrees;
    }
}
