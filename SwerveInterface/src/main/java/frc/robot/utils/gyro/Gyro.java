package frc.robot.utils.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public interface Gyro {

    public enum RotationAxis{
        yaw,
        pitch,
        roll,
    }

    public Rotation2d get(RotationAxis axis);

    public Rotation3d getRotation3d();

    public void set(Rotation2d value, RotationAxis axis);

    public void set(Rotation3d value);

    /**
     * Sets the positive direction of the gyro
     * @param isInverted The inverted value with false being CCW+ and true being CW+
     */
    public void setInverted(boolean isInverted);

    default public void zero(){
        set(new Rotation3d());
    }

    default public void update(){}
}
