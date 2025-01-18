package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Encoder {
    public Rotation2d getPos();

    public void setPos(Rotation2d postion);

    public boolean isAbsolute();

    public double getVelocity();

    public double getVoltage();

    public void setGearRatio();
}
