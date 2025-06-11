package frc.robot.utils.gyro;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class NavXGyro implements Gyro{
    private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private Rotation3d gyroValues = new Rotation3d();
    private double yawOffset, pitchOffset, rollOffset = 0;

    private double invertedValue = 1;

    private NavXGyro(){
        
    }


    @Override
    public Rotation2d get(RotationAxis axis) {
        switch (axis) {
            case yaw:
                return Rotation2d.fromDegrees(gyroValues.getZ());
            case pitch:
                return Rotation2d.fromDegrees(gyroValues.getY());
            case roll:
                return Rotation2d.fromDegrees(gyroValues.getX());
            default:
                return new Rotation2d();
        }
    }

    @Override
    public Rotation3d getRotation3d() {
        return gyroValues;
    }

    @Override
    public void set(Rotation2d value, RotationAxis axis) {
        switch (axis) {
            case yaw:
                yawOffset = value.getDegrees();
                break;
            case pitch:
                pitchOffset = value.getDegrees();
                break;
            case roll:
                rollOffset = value.getDegrees();
                break;
            default:
                break;
        }
    }

    @Override
    public void set(Rotation3d value) {
        yawOffset = value.getZ();
        pitchOffset = value.getY();
        rollOffset = value.getX();
    }

    @Override
    public void setInverted(boolean isInverted){
        invertedValue = isInverted
            ? -1
            : 1;
    }

    @Override
    public void update(){
        double yaw, pitch, roll;

        yaw = gyro.getYaw() + yawOffset;
        pitch = gyro.getPitch() + pitchOffset;
        roll = gyro.getRoll() + rollOffset;
        yaw = Units.degreesToRadians(limitNumbers(yaw));
        pitch = Units.degreesToRadians(limitNumbers(pitch));
        roll = Units.degreesToRadians(limitNumbers(roll));
        gyroValues = new Rotation3d(roll * invertedValue, pitch * invertedValue, yaw * invertedValue);
    }

    private double limitNumbers(double unlimitedValue){
        if(unlimitedValue > 180){
            unlimitedValue = -180 + (unlimitedValue % 180);
        }else if(unlimitedValue < -180){
            unlimitedValue = 180 + (unlimitedValue % -180);
        }
        return unlimitedValue;
    }

}
