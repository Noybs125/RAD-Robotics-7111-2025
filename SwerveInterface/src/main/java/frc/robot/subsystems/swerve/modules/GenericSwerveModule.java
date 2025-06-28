package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.encoder.GenericEncoder;

public interface GenericSwerveModule {
    public void setOpenDriveState(SwerveModuleState state);

    public void setClosedDriveState(SwerveModuleState state);

    public double getDriveVelocity();

    public double getDrivePosition();

    public Rotation2d getAngle();

    public void setAngle(Rotation2d rotation);

    public GenericEncoder getEncoder();

    public void zeroWheels();

    public void configure();

    default public void update(){}
}
