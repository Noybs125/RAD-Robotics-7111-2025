package frc.robot.subsystems.swerve.modules;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.config.SwerveModuleConfig;
import frc.robot.DeviceConfigs;
import frc.robot.utils.encoder.GenericEncoder;

public class SparkMaxSwerveModule implements GenericSwerveModule {

    private GenericEncoder encoder;
    private double encoderOffsetDegrees;

    private SparkMax driveMotor;
    private SparkBaseConfig driveMotorConfig = DeviceConfigs.SwerveModuleConfigs.getSparkMaxDrive(SwerveConstants.drivebaseConfig.moduleConstants[0].driveMotor);
    private RelativeEncoder driveEncoder;
    private SparkClosedLoopController drivePID;
    private SimpleMotorFeedforward driveFeedforward;

    private SparkMax angleMotor;
    private SparkBaseConfig angleMotorConfig = DeviceConfigs.SwerveModuleConfigs.getSparkMaxRotation(SwerveConstants.drivebaseConfig.moduleConstants[0].angleMotor);
    private RelativeEncoder angleEncoder;
    private SparkClosedLoopController anglePID;

    public SparkMaxSwerveModule(SwerveModuleConfig constants){
        driveMotor = new SparkMax(constants.driveMotor.id, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
        drivePID = driveMotor.getClosedLoopController();

        angleMotor = new SparkMax(constants.angleMotor.id, MotorType.kBrushless);
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        this.encoder = constants.encoder;
        encoderOffsetDegrees = constants.canCoderOffsetDegrees;
    }

    @Override
    public void setOpenDriveState(SwerveModuleState state) {
        double speed = state.speedMetersPerSecond / SwerveConstants.maxDriveVelocity;
        drivePID.setReference(speed, SparkMax.ControlType.kDutyCycle);
    }

    @Override
    public void setClosedDriveState(SwerveModuleState state) {
        drivePID.setReference(state.speedMetersPerSecond, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, driveFeedforward.calculate(state.speedMetersPerSecond));
    }

    @Override
    public double getDriveVelocity() {
        double velocity = driveEncoder.getVelocity();
        return velocity;
    }

    @Override
    public double getDrivePosition() {
        double distance = driveEncoder.getPosition();
        return distance;
    }

    @Override
    public void setAngle(Rotation2d angle){
        anglePID.setReference(angle.getRotations(), SparkMax.ControlType.kPosition);
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(angleEncoder.getPosition());
    }

    @Override
    public GenericEncoder getEncoder(){
        return encoder;
    }

    @Override
    public void zeroWheels(){
        angleEncoder.setPosition(Units.degreesToRotations((encoder.getPosition().getDegrees()) - encoderOffsetDegrees));
    }

    public void configure(){
        driveMotor.configure(driveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        angleMotor.configure(angleMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
