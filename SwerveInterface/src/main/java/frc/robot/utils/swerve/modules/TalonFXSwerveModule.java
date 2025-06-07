package frc.robot.utils.swerve.modules;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.DeviceConfigs.SwerveModuleConfigs;
import frc.robot.utils.encoder.Encoder;
import frc.robot.utils.swerve.SwerveModuleConstants;

public class TalonFXSwerveModule implements SwerveModuleType{

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private Encoder encoder;

    private DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

    private PositionVoltage anglePosition = new PositionVoltage(0);

    private double encoderOffsetDegrees;

    public TalonFXSwerveModule(SwerveModuleConstants constants){
        driveMotor = new TalonFX(constants.driveMotorConstants.id, Constants.canbus);
        angleMotor = new TalonFX(constants.angleMotorConstants.id, Constants.canbus);
        encoder = constants.encoder;

        encoderOffsetDegrees = constants.canCoderOffsetDegrees;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond / SwerveConstants.maxDriveVelocity;
        driveDutyCycle.Output = speed;
        driveMotor.setControl(driveDutyCycle);
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        var speed = state.speedMetersPerSecond;
        driveVelocity.Velocity = speed / SwerveConstants.wheelCircumference;
        driveVelocity.FeedForward = driveFF.calculate(speed);
    }

    @Override
    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.wheelCircumference;
    }

    @Override
    public double getDrivePosition() {
        return driveMotor.getPosition().getValueAsDouble() * SwerveConstants.wheelCircumference;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleMotor.getPosition().getValueAsDouble());
    }

    @Override
    public void setAngle(Rotation2d rotation) {
        anglePosition.Position = rotation.getRotations();
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }

    @Override
    public void zeroWheels() {
        angleMotor.setPosition(Units.degreesToRotations(encoder.getPosition().getDegrees() - encoderOffsetDegrees));
    }

    @Override
    public void configure() {
        driveMotor.getConfigurator().apply(SwerveModuleConfigs.getTalonFXDrive());
        angleMotor.getConfigurator().apply(SwerveModuleConfigs.getTalonFXRotation());
    }    
}
