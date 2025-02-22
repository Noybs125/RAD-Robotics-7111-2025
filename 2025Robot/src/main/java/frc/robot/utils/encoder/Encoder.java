package frc.robot.utils.encoder;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Encoder {
    /**
     * interface for getPosition on the encoders
     * @return -The position of the encoders
     */
    public Rotation2d getPosition();

    public double getPositionAsDouble();

    public void setPosition(Rotation2d position);

    public boolean isAbsolute();

    public void setGearRatio(double value);

    public double getGearRatio();

    /** true if CW, false if CCW */
    public void setDirection(boolean isCW);

    public boolean getDirection();

    /** must be called in the periodic method of the subsystem */
    public void periodic();
}
