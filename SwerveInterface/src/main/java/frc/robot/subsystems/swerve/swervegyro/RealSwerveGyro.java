package frc.robot.subsystems.swerve.swervegyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.gyro.GenericGyro;
import frc.robot.utils.gyro.NavXGyro;

public class RealSwerveGyro implements GenericSwerveGyro{
    
    private GenericGyro gyro;

    public RealSwerveGyro(GenericGyro gyro){
        this.gyro = gyro;
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getYaw();
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        gyro.setYaw(rotation);
    }

    @Override
    public void setInverted(boolean isCCW) {
        gyro.invertYaw(isCCW);
    }

    @Override
    public void update(){
        gyro.update();
    }
}
